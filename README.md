# ur_package

If you are developer, please you register github
```
git init
git commit -m "first commit"
git branch -M master
git remote add origin https://github.com/tsuchidashinya/ur_package
git pull --allow-unrelated-histories origin master
git push -u origin master
```

When you go to the trouble, please type this
```
rm -rf (name trouble repository)/.git
mv (name trouble repository) (subfoldername)/
git submodule deinit (name trouble repository)
git rm --cached (name trouble repository)
mv (subfoldername)/(name trouble repository) (name trouble repository)
git add (name trouble repository)

```

When you install, please type this command
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
