# SP2C
SP2C (Simple Physics 2D Collisions) is a simple 2d collision detection library written in C++.



There are two versions in this repository.

* SP2 (in main)

  - implements physics engine following Randygaul's [tutorials](https://tutsplus.com/authors/randy-gaul) on tutsplus and [ImpluseEngine](https://github.com/RandyGaul/ImpulseEngine) github.

  - implements Circle and Polygon (Convex).

    

*  SP2C (in SP2C directory)

  - implements only collision detections. (whether two shapes are colliding and a manifold)

  - fixed some errors from original tutorial codes.

  - implements AABB, Circle, Polygon (Convex).

  - I also implemented some custom shapes in `main_SP2C.cpp`, like RoundRect and Concaves.

    - RoundRect is a group of circles and aabbs. See functions `createRoundRect` and `drawRoundRect`.

    - Concave is converted into a group of triangles using Ear Clipping triangulation algorithm.

      Due to the generation code in main function, it can be actually a convex in random cases.

      You can set draw mode to draw each triangles, or outline of concave.

      See functions `triangulation` and `drawTriangles`

  - Only a single polygon can rotate.

    - You can test rotating polygons with the option,

      ```c++
      #define ROTATE_POLYGON_TEST 1 //line 15 in main_SP2C.cpp
      ```

    - AABB (as it is axis-aligned) and Circles, for sure, don't rotate.
    - I didn't implement polygon rotation in the concave (group of triangles).



- Others

  - used [SFML Library](https://www.sfml-dev.org/) to render graphics.

  - environment : C++17, Visual Studio 2019, SFML 2.5.1

  - You can build `main_SP2C.cpp`  with defining in `test_def.h`,

    ```
    #define __MAIN_SP2C__
    ```

    Or you can build `main.cpp` undefining the option.

  - Please let me know if there is a problem or something to fix, via gmail.

