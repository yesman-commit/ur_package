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

When you install, please type this command
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
