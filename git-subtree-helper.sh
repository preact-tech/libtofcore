#!/bin/bash
SOURCE_BRANCH="libtofcore_internal-source"
STAGING_BRANCH="libtofcore_internal-staging"

echo
echo "Fetch new changes from libtofcore_internal"
echo -----------------------------------------
git fetch libtofcore_internal release/1.1

LIBTOFCORE_LATEST_COMMIT=`git ls-remote libtofcore_internal | grep "refs/heads/release/1.1" | awk '{ print $1}'`
echo
echo "libtofcore_internal latest commit: ${libtofcore_internal_LATEST_COMMIT}"
echo

# checkout source repo
git checkout -b ${SOURCE_BRANCH} libtofcore_internal/release/1.1

ls
# create new staging branch from all the commits impacting "/my-chart" from source repo
git subtree split -P tofcore -b ${STAGING_BRANCH}

git add .vscode/settings.json
git commit -m "hacky fix"
# checkout develop
git checkout main

# take commits in the staging branch and set "/helm-charts/my-chart" as the commit root
# after you run this script the first time, update the command below to:
# "git subtree merge" with the same parameters (previously was subtree add)  
#
echo
echo "Add in changes"
echo -----------------------------------------
git subtree merge -P tofcore ${STAGING_BRANCH} --message "Update tofcore from internal repo, commit: ${LIBTOFCORE_LATEST_COMMIT}"



# clean up
echo
echo
git branch -D ${STAGING_BRANCH}
git branch -D ${SOURCE_BRANCH}
