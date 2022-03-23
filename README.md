# SP2C
SP2C (Simple Physics 2D Collisions) is a simple 2d collision detection library written in C++.



![main_image](https://user-images.githubusercontent.com/26345945/159725133-2c1e9574-4a68-4fdf-befa-155dde2f9a22.png)



### Contents


There are two versions in this repository.

* SP2 (in main)

  - implements physics engine following Randygaul's [tutorials](https://tutsplus.com/authors/randy-gaul) on tutsplus and [ImpulseEngine](https://github.com/RandyGaul/ImpulseEngine) github.

  - implements Circle and Polygon (Convex).

    

* SP2C (in SP2C directory)

  - implements only collision detections. (whether two shapes are colliding and a manifold)

  - fixed some errors from original tutorial codes.

  - implements AABB, Circle, Polygon (Convex).

    - Polygon automatically find the hull points in counter-clockwise order.

      You can turn off with setting bool parameter `ordering` as false and put the vertices directly.

      See `Set` function in struct `SP2C::SPC_Polygon` in `SPC_Shapes.h`.

      Also you can create a box polygon with function `SetBox`. Use it when you need rotating boxes.

  - I also implemented some custom shapes in `main_SP2C.cpp`, like RoundRect and Concaves.

    - RoundRect is a group of circles and aabbs. See function `createRoundRect` and `drawRoundRect`.

      You can set draw mode to draw each circles and boxes, or outline of roundrect.

      See function `createRoundRect` and `drawRoundRect`.

    - Concave is converted into a group of triangles using Ear Clipping triangulation algorithm.
  
      Due to the generation code in main function, it can be actually a convex in random cases.

      You can set draw mode to draw each triangles, or outline of concave.
  
      See functions `triangulation` and `drawTriangles`.
  
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

    ```c++
    #define __MAIN_SP2C__
    ```

    Or you can build `main.cpp` undefining the option.

  - Please let me know if there is a problem or something to fix, via gmail.



### Example


```c++
#include "SPC_Shapes.h"
#include "SPC_Collision.h"

SP2C::SPC_AABB box1;
box1.min = SP2C::Vec2(100, 150);
box1.max = SP2C::Vec2(200, 300);

SP2C::SPC_Circle circle1;
circle1.radius = 30;
circle1.position = SP2C::Vec2(215, 320);

bool collided1 = SP2C::Collision::AABB_to_Circle(box1, circle1);

SP2C::SPC_AABB box2;
box2.min = SP2C::Vec2(400, 150);
box2.max = SP2C::Vec2(450, 300);

SP2C::SPC_AABB box3;
box3.min = SP2C::Vec2(470, 180);
box3.max = SP2C::Vec2(520, 220);

bool collided2 = SP2C::Collision::AABB_to_AABB(box2, box3);
```

The result would be like:

![image_1](https://user-images.githubusercontent.com/26345945/159725306-675ef833-3c34-4cc4-b01b-310ffb4c656b.png)



Also, you can use manifold to do same thing.

```c++
SP2C::SPC_AABB box1;
box1.min = SP2C::Vec2(100, 150);
box1.max = SP2C::Vec2(200, 300);

SP2C::SPC_Circle circle1;
circle1.radius = 30;
circle1.position = SP2C::Vec2(215, 320);

SP2C::SPC_Manifold m;
m.A = &box1;
m.B = &circle1;

bool collided = SP2C::Collision::AABB_to_Circle(&m);
```

The function `SP2C::Collision::Collide` will detect manifold's shape types.

```c++
bool collided = SP2C::Collision::Collide(&m);
```



You can get contact info (contact points, contact counts, normal and penetration) from manifold.

```c++
if (SP2C::Collision::Collide(&m))
{
	SP2C::Vec2 p1 = m.contact_points[0];
	SP2C::Vec2 p2 = p1 + m.normal * m.penetration;
}
```

![image_2](https://user-images.githubusercontent.com/26345945/159725399-bb04cb00-1287-4724-a2a4-dfcd6b702ccd.png)



A vector `m.normal * m.penetraion` indicates the line (from green to blue) from the shape A.

You can use it like the code below.

```c++
SP2C::Vec2 f = -m.normal * m.penetration; //invert direction for restitution

switch (m.A->type)
{
	case SP2C::ShapeType::AABB:
		reinterpret_cast<SP2C::SPC_AABB*>(m.A)->Translate(f.x, f.y);
        break;

    case SP2C::ShapeType::Circle:
	    reinterpret_cast<SP2C::SPC_Circle*>(m.A)->position += f;
    	break;

    case SP2C::ShapeType::Polygon:
		reinterpret_cast<SP2C::SPC_Polygon*>(m.A)->Translate(f.x, f.y);
		break;
		
	default:
		break;
}
```

The result would be like:

![image_3](https://user-images.githubusercontent.com/26345945/159725513-9bc72251-dcec-4919-b6fa-443b3edcb41b.gif)



