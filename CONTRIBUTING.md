Contributing
============

👍🎉 Thanks a lot for considering contributing 🎉👍

We welcome and encourage contribution. There is many way to contribute: you can
write bug report, contribute code or documentation (please, take a look to the 
[Wiki section](https://github.com/gsilano/CrazyS/wiki)).

## Working on your first Pull Request?

You can learn how from this *free* series [How to Contribute to an Open Source Project on GitHub](https://egghead.io/series/how-to-contribute-to-an-open-source-project-on-github)

## Reporting issues

When reporting issues the more information you can supply the better.

If it is an issue with building the code, indicate your environment (Indigo Igloo or Kinetic Kame versions of ROS)
like operating system (Ubuntu or MacOS).

## Improvements request and proposal

Feel free to make an issue to request a new functionality.

## Contributing code/Pull-Request

We welcome contribution, this can be done by starting a pull-request.
You can contribute by updating the ROS package content or styling, code, or both.

### Contributing content or styling

There is a couple of basic requirement for us to merge the pull request:
 - The pull request must pass the automated test (a TravisCI script will check if everythings is OK)

### Contributing code

If the change is big, typically if the change span to more than one file, consider starting an issue first to discuss the improvement.
This will makes it much easier to make the change fit well into the software.

There is some basic requirement for us to merge a pull request:
 - Describe the change (a bot will remind you in case you forget it)
 - Refer to any issues it effects
 - Separate one pull request per functionality: if you start writing "and" in the feature description consider if it could be 
 separated in two pull requests.
 - The pull request must pass the automated test (as described previously)

In your code:
 - Make sure the coding style of your code follows the style of the file (take a look at the [Developer Section](https://github.com/gsilano/CrazyS/wiki#for-developers) into the first page of the CrazyS Wiki page).
 
## Workflow

### Stable branch: `master`

The `master` branch is stable and **should not receive new features**. Only **bug fixes** are accepted.

This is the typical workflow to fix a bug in the master branch.

* Identify a bug that does not require breaking changes of the API/ABI.
* Open an issue on github.
* Add some label (FIXME which label?).
* Assign the issue to yourself.
* Create a new branch starting from the `master` branch:

```
git fetch origin
git checkout -b <branch_name> origin/master
```

* Fix the bug and make one or more commits.
* [Push the branch on your fork and create a pull request](https://help.github.com/categories/collaborating-on-projects-using-pull-requests/).
* Wait for someone else to review your fix and merge your pull request.
* Your fix is now in the `master` branch, now you need to port it to the `dev`
  branch.
* Ensure that your branches are in sync with `origin`:

```
git checkout master
git pull --rebase origin master
git checkout dev
git pull --rebase origin dev
```

  * Merge master into devel and eventually fix the conflicts.

```
git merge master
```

##### Work in progress PR
As final note, in case you need to start a PR but you deem it still **work-in-progress** and do not want anyone to merge it by mistake, do the following:
- Put `[WIP]` at the beginning of the PR title.
- Mark the PR with the label `"Status: In Progress"`.

Once you are happy about your work, just remove the `[WIP]` tag as well as the label, and drop a message within the PR to notify the community that reviews are welcome and merging is now possible.

### Development branch: `dev`


We use the branch `dev` to collect the ongoing work, which is given in terms of **new features** and **bug fixes**.

When we introduce a new feature that will cause downstream projects to be aware of such update, we do increase the tweak number (always sticking to _odd numbers_).

When we decide to publish these new features in a new software release (roughly each _3 months_), we merge the new modifications into `master`, doing:

```sh
git checkout master
git merge --no-ff dev
git push origin master
```

