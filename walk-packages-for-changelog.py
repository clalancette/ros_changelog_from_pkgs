# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is a program to generate an exhaustive set of changes in a ROS source
# tree.  This works by crawling the given source tree, building up a database
# of packages.  It also looks at an input ros2.repos file, and builds another
# database of repository names to version numbers.  For each of the packages,
# it looks at all of the changes to the CHANGELOG.rst file for that package
# since the earlier ros2.repos file, and collates all of them together into
# a single set of changes.  All of the collated changelog information is written
# to the given output_file.  If a repository doesn't have a CHANGELOG.rst file,
# this fact is noted and then printed to stdout at the end.

import argparse
import os
import re
import subprocess
import sys

import lxml.etree
import yaml


def get_repo_path_from_package_path(package_path):
    curpath = os.path.realpath(package_path)
    while not os.path.exists(os.path.join(curpath, '.git')) and curpath != '/':
        curpath = os.path.realpath(os.path.join(curpath, '..'))

    if curpath == '/':
        raise Exception("Could not find git repository")

    return curpath

def remove_duplicate_slashes(instring):
    outlist = []
    last = ''
    for c in instring:
        if c != '/':
            outlist.append(c)
        else:
            if last != '/':
                outlist.append(c)

        last = c

    return ''.join(outlist)

def get_contrib_from_line(instring):
    contributors = []
    contrib_match = re.match(r'^\* Contributors: (.*)', instring)
    if contrib_match:
        groups = contrib_match.groups()
        if len(groups) == 1:
            # OK, we have a list of contributors.  Split this up so
            # that we can do de-duplication, sorting, etc.
            s = groups[0].split(',')
            for contributor in s:
                contributors.append(contributor.strip())

    return contributors

# This is a very complicated function to apply a set of transformations to a line.
# It ended up this complicated because I could not convince Python regular
# expressions to do all of these transformations in a non-greedy manner, but I'd
# be happy to trade all this complexity in for a smaller set of regular expressions
# that work.
#
# What this function is doing is applying the following RST rules to each line:
# 1. If a string contains ``foo``, that is a correctly emphasized word and we
#    leave it alone.
# 2. If a string contains `foo`, that is an incorrectly emphasized word and we
#    need to double up the `.
# 3. If a string contains `foo <>`_, that is hyperlink that is "non-anonymous".
#    That is, the word "foo" is now an anchor that can be referenced elsewhere.
#    However, since this is all auto-generated changelogs and we don't control
#    the input, we can definitely have collisions.  So we replace the single
#    underscore with a double-underscore, which makes it anonymous.
# 4. If a string contains `foo <>`__, it is already anonymous so we leave it alone.
# 5. Everything else we need to leave alone.
def fix_line(instring):
    # Rule 5; if there is no '`' in the string, there is no transformation to be done
    if not '`' in instring:
        return instring

    # First, double up *all* ticks.  This will make some triple ticks from things
    # that already have double ticks.
    double_and_triple_ticks = re.sub(r'`(?:([^`])|$)', r'``\1', instring)

    # Now replace triple-ticks with double-ticks.
    double_ticks = re.sub(r'```', r'``', double_and_triple_ticks)

    # Now double up all underscores after links
    double_underscore = re.sub(r'`_(?:([^_])|$)', r'`__\1', double_ticks)

    # And finally convert ``link <link>``__ to `link <link>`__.  We walk the
    # string character-by-character and use a state machine to translate it
    # as necessary.
    REGULAR_CHAR_STATE = 0
    SAW_ONE_LEADING_TICK_STATE = 1
    SAW_TWO_LEADING_TICKS_STATE = 2
    SAW_ONE_TRAILING_TICK_STATE = 3
    SAW_TWO_TRAILING_TICKS_STATE = 4
    state = REGULAR_CHAR_STATE

    output = ''
    tickbuf = ''
    for c in double_underscore:
        if state == REGULAR_CHAR_STATE:
            if c == '`':
                state = SAW_ONE_LEADING_TICK_STATE
            else:
                output += c
        elif state == SAW_ONE_LEADING_TICK_STATE:
            if c == '`':
                state = SAW_TWO_LEADING_TICKS_STATE
            else:
                output += '`' + c
                state = REGULAR_CHAR_STATE
        elif state == SAW_TWO_LEADING_TICKS_STATE:
            if c == '`':
                # This is here to handle the case where we have more than 2
                # consecutive ticks
                if not tickbuf:
                    output += '`'
                else:
                    state = SAW_ONE_TRAILING_TICK_STATE
            else:
                tickbuf += c
        elif state == SAW_ONE_TRAILING_TICK_STATE:
            if c == '`':
                state = SAW_TWO_TRAILING_TICKS_STATE
            else:
                output += '``' + tickbuf + '`'
                tickbuf = ''
                state = REGULAR_CHAR_STATE
        elif state == SAW_TWO_TRAILING_TICKS_STATE:
            if c == '_':
                # OK, this is a link.  We need to go back and put the whole thing in
                # place with single ticks
                output += '`' + tickbuf + '`_'
            else:
                output += '``' + tickbuf + '``' + c

            tickbuf = ''
            state = REGULAR_CHAR_STATE

    if tickbuf:
        output += '``' + tickbuf + '``'

    return output.rstrip()

def get_old_repo_versions(since):
    old_repo_versions = {}
    with open(since, 'r') as infp:
        old_repos = yaml.safe_load(infp)
        for repo,info in old_repos['repositories'].items():
            old_repo_versions[os.path.basename(repo)] = info['version']
    return old_repo_versions

def get_package_name_from_xml(dirpath):
    lxml_tree = lxml.etree.parse(os.path.join(dirpath, 'package.xml'))
    package_name = ''
    for child in lxml_tree.getroot().getchildren():
        if child.tag == 'name':
            package_name = child.text
            break
    return package_name

def has_skip_file(dirpath):
    should_skip = False
    for skip_file in ('COLCON_IGNORE', 'AMENT_IGNORE', 'CATKIN_IGNORE'):
        skip_path = os.path.join(dirpath, skip_file)
        if os.path.exists(skip_path):
            should_skip = True
            break

    return should_skip

def get_old_version(repo_path, old_repo_versions):
    repo_name = os.path.basename(repo_path)
    if repo_name in old_repo_versions:
        # This was a package that existed in Foxy, so get the diff since Foxy
        old_version = old_repo_versions[repo_name]
    else:
        # This is a new package, so get the diff since the beginning of time
        p = subprocess.Popen(['git', 'rev-list', 'HEAD'], cwd=repo_path, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output,err = p.communicate()
        rc = p.returncode
        if rc != 0:
            old_version = None
        else:
            old_version = output.decode('utf-8').splitlines()[-1]

    return old_version

def get_origin_url(dirpath):
    p = subprocess.Popen(['git', 'config', '--get', 'remote.origin.url'],
                         cwd=dirpath, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output,err = p.communicate()
    rc = p.returncode
    origin_url = ''
    if rc == 0:
        origin_url = output.decode('utf-8').strip()

    return origin_url

def get_current_branch(dirpath):
    p = subprocess.Popen(['git', 'branch', '--show-current'],
                         cwd=dirpath, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output,err = p.communicate()
    rc = p.returncode
    current_branch = ''
    if rc == 0:
        current_branch = output.decode('utf-8').strip()

    return current_branch

def get_changelog(dirpath, old_version):
    star_fix_re = re.compile(r'\*\.')

    p = subprocess.Popen(['git', 'diff', '-U0', '--output-indicator-new', ' ', old_version + '..', 'CHANGELOG.rst'],
                         cwd=dirpath, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output,err = p.communicate()
    rc = p.returncode
    if rc != 0:
        return None

    # The output from subprocess.Popen.communicate() is a bytestring, so
    # we first need to decode that into utf-8
    changelog_string = output.decode('utf-8')

    # The output string that we now have comes from the 'git diff' command
    # with a diff header, and with a single leading space on each line.  That
    # is, it looks like:
    #
    # diff --git a/pkg/CHANGELOG.rst b/pkg/CHANGELOG.rst
    # index aaaaaaaa..bbbbbbbb 100644
    # --- a/pkg/CHANGELOG.rst
    # +++ b/pkg/CHANGELOG.rst
    # @@ -4,0 +5,284 @@
    #  changelog line 1
    #  changelog line 2
    #  etc
    #
    # Note that depending on how we obtained the diff, the header may be slightly
    # different.
    #
    # We want to both get rid of that git diff header, and remove the
    # leading space on each line.  So we split the string up into lines,
    # skip lines until the first one with a leading space, and then
    # strip the space from the rest and store it.

    changelog_list = []
    contributors = []
    line_buffer = ''
    for line in changelog_string.splitlines():
        if len(line) == 0 or line.isspace():
            continue

        # It is possible that there are lines in the diff output that don't
        # correspond to this release; this can happen if the CHANGELOG.rst is
        # brand-new, for instance.  While we are looking at each individual
        # line, if the number matches the 'old_version', we know we are done.
        if re.match(r' %s \([0-9]{4}-[0-9]{2}-[0-9]{2}\)' % (old_version), line):
            break

        if len(line) > 1:
            if line[0:2] == ' *':
                # Flush out the previous buffer
                if line_buffer:
                    current_contrib = get_contrib_from_line(line_buffer)
                    if current_contrib:
                        contributors.extend(current_contrib)
                    else:
                        changelog_list.append(fix_line(line_buffer))

                line_buffer = re.sub(star_fix_re, r'\*.', line.lstrip())
            elif line[0:2] == '  ':
                if line_buffer:
                    line_buffer += ' ' + re.sub(star_fix_re, r'\*.', line.lstrip())

    if line_buffer:
        # Flush out the last buffer
        current_contrib = get_contrib_from_line(line_buffer)
        if current_contrib:
            contributors.extend(current_contrib)
        else:
            changelog_list.append(fix_line(line_buffer))

    if len(changelog_list) == 0:
        # if there were no entries for a particular package, just skip
        # aggregating anything about it.
        return None

    if contributors:
        changelog_list.append('* Contributors: ' + ', '.join(sorted(set(contributors))))
    cooked_changelog = '\n'.join(changelog_list)
    cooked_changelog += '\n\n\n'

    return cooked_changelog

def main():
    parser = argparse.ArgumentParser(description='Utility to find and collate CHANGELOGs of packages in a workspace')
    parser.add_argument(
        'source_path',
        help='The top-level of the source tree in which to find packages',
        action='store')
    parser.add_argument(
        'since',
        help='The ros2.repos file to generate documentation since',
        action='store')
    parser.add_argument(
        'output_file',
        help='The location in which to write long-form output',
        action='store')
    args = parser.parse_args()

    # Open up the old ros2.repos file, and store a dict of the package_name -> tag
    # for each package.
    old_repo_versions = get_old_repo_versions(args.since)

    # Clear out the old file that we'll be overwriting
    with open(args.output_file, 'w') as outfp:
        header = 'ROS 2 Galactic Geochelone Complete Changelog'
        outfp.write(header + '\n')
        outfp.write('='*len(header) + '\n\n')
        outfp.write('This page is a list of the complete changes in all ROS 2 core packages since the previous release.\n\n')
        outfp.write('.. contents:: Table of Contents\n')
        outfp.write('   :local:\n\n')

    # Walk the entire source_path passed in by the user, looking for all of the
    # package.xml files.  For each of them we parse the package.xml, and go
    # looking for changelog.

    packages_with_no_changelog = []
    for (dirpath, dirnames, filenames) in os.walk(args.source_path):
        if 'package.xml' not in filenames:
            continue

        if has_skip_file(dirpath):
            continue

        package_name = get_package_name_from_xml(dirpath)
        if not package_name:
            print('Odd, package has no name; skipping')
            continue

        changelog = os.path.join(dirpath, 'CHANGELOG.rst')
        if not os.path.exists(changelog):
            packages_with_no_changelog.append(package_name)
            continue

        repo_path = get_repo_path_from_package_path(dirpath)

        old_version = get_old_version(repo_path, old_repo_versions)
        if old_version is None:
            packages_with_no_changelog.append(package_name)
            continue

        origin_url = get_origin_url(dirpath)

        current_branch = get_current_branch(dirpath)

        cooked_changelog = get_changelog(dirpath, old_version)
        if cooked_changelog is None:
            packages_with_no_changelog.append(package_name)
            continue

        header = 'Changelog for package '
        if origin_url and origin_url.startswith('https://github.com'):
            if origin_url.endswith('.git'):
                origin_url = origin_url[:-4]
            url_path = '/'
            if len(dirpath) != len(repo_path):
                url_path = dirpath[len(repo_path):]
            url = remove_duplicate_slashes('%s/tree/%s/%s/CHANGELOG.rst' % (origin_url, current_branch, url_path))
            header += '`%s <%s>`__' % (package_name, url)
        else:
            header += package_name

        with open(args.output_file, 'a+') as outfp:
            outfp.write('^'*len(header) + '\n')
            outfp.write(header + '\n')
            outfp.write('^'*len(header) + '\n\n')
            outfp.write(cooked_changelog)

    if packages_with_no_changelog:
        print("Packages without a changelog:")
        for package_name in sorted(packages_with_no_changelog):
            print('* [ ] %s' % (package_name))

    return 0


if __name__ == '__main__':
    sys.exit(main())
