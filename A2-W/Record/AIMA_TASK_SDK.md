# AIMA_TASK_SDK DESCRIP

## 沙盒子打包

`aima_task_sdk_example/build.sh`

用于自动化构建、编译和打包一个项目，同时生成一个包含开发环境的沙盒压缩包。

> ```bash
> cd $(dirname $0)
> prj_dir=$(pwd)
> ```
>
> 将工作目录切换到脚本所在位置，并保存为`prj_dir`变量（项目根目录）
>
> ```bash
> cmake -B build -DCMAKE_BUILD_TYPE=Release
> if [ $? -ne 0 ]; then
>     echo "cmake failed"
>     exit 1
> fi
> 
> cmake --build build -j 8
> if [ $? -ne 0 ]; then
>     echo "cmake build failed"
>     exit 1
> fi
> ```
>
> **CMake配置**：在`build`目录下生成编译文件（`-B build`），并指定为**发布模式**（`-DCMAKE_BUILD_TYPE=Release`，优化代码且不包含调试信息）。
>
> **编译项目**：使用8个线程并行编译（`-j 8`）以加快速度。
>
> **错误处理**：若任一步骤失败，脚本立即终止并报错。
>
> * `[ $? -ne 0 ]`**：检查上一个命令的退出状态码是否**不等于 0
>
> ```bash
> cd build || exit 1
> make -j$(nproc)
> if [ $? -ne 0 ]; then
>     echo "make failed"
>     exit 1
> fi
> 
> make install
> ```
>
> **切换到构建目录**：执行编译生成的`Makefile`。
>
> **全核编译**：`-j$(nproc)`自动使用所有CPU核心。
>
> + `-j`：启用并行编译模式，后接数字表示线程数。若省略数字，默认使用所有可用核心。
> + `$(nproc)`：
>    ​**动态获取 CPU 核心数**。例如：
>   - 在 4 核 CPU 系统中，`$(nproc)` 返回 `4`
>   - 在 8 核 CPU 系统中，`$(nproc)` 返回 `8`
>
> **安装程序**：将编译好的文件安装到系统目录（如`/usr/local/bin`）。
>
> ```bash
> ZIP_FILE="task_sandbox.zip"
> mkdir -p "task_sandbox"
> cp -r ./agi_task_sdk/install/* "task_sandbox/"
> zip -r "${ZIP_FILE}" "task_sandbox/"
> rm -rf "task_sandbox"
> ```
>
> **创建沙盒目录**：`task_sandbox`用于存放SDK环境。
>
> **复制SDK文件**：将`agi_task_sdk/install`下的内容复制到沙盒目录。
>
> **生成ZIP包**：压缩沙盒目录为`task_sandbox.zip`。
>
> **清理临时文件**：删除沙盒目录以节省空间。



> `static_assert`:
>
>  C++11 引入的一种编译时断言机制，用于在编译期间验证常量表达式。如果断言条件不满足，编译器会生成一条错误信息，并导致编译失败。
>
> ```
> static_assert(条件, "错误信息");
> ```


