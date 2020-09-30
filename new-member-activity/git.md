# Creating a GitHub pull request

In this activity, you will be introduced to our GitHub workflow.
This will walk you through creating a pull request.
Pull requests are how members contribute code to our repositories.

If you're not familiar with git, here are a couple of pages to check out:

- https://guides.github.com/introduction/git-handbook/
- https://guides.github.com/introduction/flow/

## Overview of Workflow

This is a brief overview of our GitHub workflow.
You can come back to this as a reference later.

1. Create a new branch to hold code for a new feature.
2. Create a commit and push that commit to github.
3. Open a pull request on github.
4. Address any comments people make on your pull request.
5. Once all comments have been resolved, someone will merge your pull request
   into the master branch.

For this activity, we will go through steps 1-3 but not 4 and 5.

## 1. Creating a feature branch

Branches are a way of keeping track of different versions of a repo.
When working on a new feature, you should create a new branch

To create a new branch, first open up a terminal and navigate to the
`spear-turtlebot` directory on your computer.

You can use the `git status` command to see which branch you currently have
checked out.

Run `git status` and you should see output like this (if you have edited the
files from the publisher/subscriber activity).

```
$ git status
On branch master
Your branch is up to date with 'origin/master'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   publisher_subscriber/package.xml
	modified:   publisher_subscriber/publisher_subscriber/publisher_member_function.py
	modified:   publisher_subscriber/publisher_subscriber/subscriber_member_function.py
	modified:   publisher_subscriber/setup.py

no changes added to commit (use "git add" and/or "git commit -a")
```

Now let's create a new branch. The basic syntax for creating a new git branch
looks like this.
You can replace `<feature name>` with a short description of the feature you're
working on and `<your name>` with your first name.

```
git checkout -b <feature name>-<your name>
```

Since we will be creating a branch for the publisher and subscriber nodes we
created in the previous activity, let's name this branch
`publisher-subscriber`.
For this activity, the command that you run should look something like this (of
course, with your own name).

```
git checkout -b publisher-subscriber-ryan
```

## 2. Create a commit

Next, run the `git add` command below.
This command is used to tell git which files we want to commit.
The `-A` flag tells git to add all the files that have been modified.

```
git add -A
```

Next we will create a commit.
A commit is essentially a snapshot of the changes we have made.
Every commit requires a message.
This message can be added using the `-m` flag.

```
git commit -m "Add simple publisher and subcriber"
```

At this point, the local copy of the git repo will have the new commit but the
remote repo on github's servers will not.
To get the changes you made onto github, we must make a push:

```
git push -u origin <name of your branch>
```

Now your commit and new branch have been sent to github.

## 3. Open a pull request on github.

Next, go to the github repo in your browser: https://github.com/UofA-SPEAR/spear-turtlebot

Click on the "Pull Requests" tab, then click on the "New Pull Request" button.

Set "base: master" and "compare: <name of your branch>".
This means that you are requesting the commits from your branch to be applied
to master.

Once you have set the base and compare branches, click "Create pull request".

Normally, at this point someone would review your pull request and possibly
suggest changes before merging it into the master branch.
