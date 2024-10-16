## mrqt

1. 修改mrpt2.spec 文件

   新增：

   ```bash
   %define ros_distro humble
   ```

   修改：

   ```
   Requires: libglfw3-dev --> Requires: glfw-devel
   Requires: libxxf86vm   -->  Requires: libXxf86vm-devel
   BuildRequires: libxxf86vm --> BuildRequires: libXxf86vm-deve
   BuildRequires: libopenni2-dev --> BuildRequires: openni-devel
   BuildRequires: wx-common --> BuildRequires: wxGTK3-devel
   ```

2. EulerMaker 构建当前报错

   ```
   Error: Unable to find a match: bzip2wx 
   No matching package to install: 'glfw-devel'
   No matching package to install: 'libfreenect-devel'
   ```

   原始报错：

   ```
   No matching package to install: 'libfreenect-devel'
   No matching package to install: 'libglfw3-devel'
   No matching package to install: 'libxxf86vm'
   No matching package to install: 'openni2-devel'
   No matching package to install: 'wx-common'
   No matching package to install: 'wxwidgets'
   ```

3. 导入包libfreenect-0.6.4-1.fc37 构建失败 ：

   ```
   Error: Unable to find a match: bzip2wx
   ```

4. 导入包glfw-3.3.7-1.fc37 构建失败 ：

   ```
   Error: Unable to find a match: bzip2wx
   No matching package to install: 'opencv-devel'
   ```

   