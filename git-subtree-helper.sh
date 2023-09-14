#!/bin/bash
SOURCE_BRANCH="libtofcore-source"
STAGING_BRANCH="libtofcore-staging"

echo
echo "Fetch new changes from libtofcore"
echo -----------------------------------------
git fetch libtofcore develop

LIBTOFCORE_LATEST_COMMIT=`git ls-remote libtofcore | grep "refs/heads/develop" | awk '{ print $1}'`
echo
echo "libtofcore latest commit: ${LIBTOFCORE_LATEST_COMMIT}"
echo

# checkout source repo
git checkout -b ${SOURCE_BRANCH} libtofcore/develop

ls
# create new staging branch from all the commits impacting "/my-chart" from source repo
git subtree split -P tofcore -b ${STAGING_BRANCH}

# checkout develop
git checkout test

# take commits in the staging branch and set "/helm-charts/my-chart" as the commit root
# after you run this script the first time, update the command below to:
# "git subtree merge" with the same parameters (previously was subtree add)
#
echo
echo "Add in changes"
echo -----------------------------------------
git subtree add -P test ${STAGING_BRANCH} --message "Update helmchart from https://github.com/icheko/libtofcore/commit/${LIBTOFCORE_LATEST_COMMIT}"

# clean up
echo
echo
git branch -D ${STAGING_BRANCH}
git branch -D ${SOURCE_BRANCH}
