#!/bin/bash -e

# Create a release tag

readonly script=$(basename "$0")
if [ "$#" -ne 2 ]; then
    echo "Usage $script <OSRAM-base-release> <BRAINLAB-consecutive-release>"
    echo "<OSRAM-base-release> - the OSRAM upstream repo release on which we are based"
    echo "<BRAINLAB-consecutive-release> - the BRAINLAB consecutive number ontop of osram release, must be [0..9] one digit"
    echo "Call script only from release branch!"
    exit 1
fi

readonly osram_version=$1
readonly brainlab_version=$2

if ! git diff-index --quiet HEAD ; then
    # Note: this ignores untracked files!
    echo "Git working directory not clean!"
    exit 1
fi

if ! echo "$brainlab_version" | grep -P -q "^[0-9]$" ; then
    echo "Brainlab Version $brainlab_version malformed, see help."
    exit 1
fi

if ! git rev-parse --abbrev-ref HEAD | grep -P -q "release"; then
    echo "Repository is not on release branch!"
    exit 1
fi

readonly release_version="${osram_version}_brainlab${brainlab_version}"
readonly release_branch="release/${release_version}"

if ! git rev-parse --abbrev-ref HEAD | grep -P -q "${release_branch}"; then
    echo "Repository is not on expected release branch: ${release_branch}"
    exit 1
fi

if ! git push --set-upstream origin "$release_branch" ; then
    echo "Failed to push release branch $release_branch"
    exit 1
fi

git tag "${release_version}"
git push origin "${release_version}"
