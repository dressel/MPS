# MPS

## Configuration File
---------------------
```
# This is a comment
search_size,100
# This is another comment
vehicle,xy,50,-50,limits,0,100,-100,0,cells,11,11
sensor,bo,13
filter,df,31,1
policy,1,0
```
Any line starting with a pound sign is a comment and disregarded.

The search size line begins with `search_size`.
It must be immediately followed by the size of one side of the search area, in meters.
In the above example, a 100m x 100m search area is created.

The vehicle line beings with `vehicle` and is optional.

The policy extra line must start with `policy`.
