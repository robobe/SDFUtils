# SDFUtils
Gazebo SDF utils

## XACRO
- properties
- macro
- include
- pure python

## properties
- declare variable to use by other sdf section


##### source sdf with xacro tags
```xml
<sdf xmlns:xacro="http://xacro">
    <xacro:property name="width" value=".2" />
    <xacro:property name="bodylen" value=".6" />
    <xacro:property name="x" value="1" />
    <xacro:property name="y" value="2" />
    <xacro:property name="z" value="3" />
    <link name="base_link">
        <pose>${x} ${y} ${z} 0 0 0</pose>
        <visual>
            <geometry>
                <cylinder radius="${2*width}" length="${bodylen}" />
            </geometry>
        </visual>
    </link>
</sdf>
```
##### output sdf
```xml
<sdf >
    <link name="base_link">
        <pose>1.0 2.0 3.0 0 0 0</pose>
        <visual>
            <geometry>
                <cylinder radius="0.4" length="0.6"/>
            </geometry>
        </visual>
    </link>
</sdf>
```

## include
include other `xacro` files to use

- `file://` : load files relative to sdf/xacro file
- `package://` : load files from `helper` folder under script source folder

```xml
<xacro:include uri="file://data.xacro" />
<xacro:include uri="package://common.xacro" />
<xacro:include uri="package://color.xacro" />
```

- `data.xacro`: custom file hold data properties for example
- `common.xacro`: common macro's like inertia and basic geometry
- `color.xacro`: color and texture macro's
