看总文件中的CMakeLists:
分别将 可执行文件 生成到指定目录中去 SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) ， 中途编译出的库文件 生成到指定目录中去 SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
将自己编写的头文件引用到文件中去INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)
将上方编译出的库文件链接LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
指定源文件夹，在源文件夹中继续构建ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)

src/:
CMakeLists:
寻找库的路径，添加依赖库FIND_PACKAGE( PCL 1.7 REQUIRED ) FIND_PACKAGE( OpenCV 3 REQUIRED )

但是这个add_definitions什么意思
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

引入设定好的库目录 INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )

将自己写的库引入到源文件上，target那个是添加源文件的依赖库
```
ADD_LIBRARY(slambase_shared SHARED slamBase.cpp)
TARGET_LINK_LIBRARIES(slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )
 ```       

最后生成可执行文件
ADD_EXECUTABLE( pointCloudFusion pointCloudFusion.cpp )
TARGET_LINK_LIBRARIES(pointCloudFusion
        slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )

---

通常情况下，写一个小的程序
先写引用头文件 #include... 命名空间 struct class typedef 函数原型 宏定义等。
再写主函数 int main(){}
最后写函数定义

但是在大的工程下，上述第一步会直接写入以整个头文件中，如headerfile.hpp
主函数直接写为 source.cpp ，该源文件引用上述头文件headerfile.hpp
最后函数定义可写为库文件，而库文件是从headerfile.cpp文件转化成的,headerfile.cpp只是规定和头文件同名，该文件包含函数定义
注意：后两个都需要引入头文件headerfile.hpp


---

看一个新的工程，首先，先看CMakeLists，从而获知工程文件夹构成、含义
再看主函数，也就是源文件。
再从源文件下手，看其他头文件，库的源文件等

---

当程序需要输入多个外部路径时，先写一个总文件夹，`string path = "****/"`
然后再根据具体需要，分写其他文件 `string cameraPosePath = path + "*.txt";
string rgbpath = path +"rbg/rgb" +to_string(idx) +".png";`












