# SP2C
SP2C (Simple Physics 2D Collisions) is a simple 2d collision detection library written in C++.



![main_image](https://user-images.githubusercontent.com/26345945/159725133-2c1e9574-4a68-4fdf-befa-155dde2f9a22.png)

### References
- Randygaul's [tutorials](https://tutsplus.com/authors/randy-gaul) on tutsplus
- [ImpulseEngine](https://github.com/RandyGaul/ImpulseEngine) github

### Contents

SP2C

  - implements only collision detections. (whether two shapes are colliding and a manifold)

  - implements AABB, Circle, Polygon (Convex).

    - Polygon automatically finds the hull points in counter-clockwise order.

      You can turn off with setting bool parameter `ordering` as false and put the vertices directly.

      See function `Set` in struct `SP2C::SPC_Polygon` in `SPC_Shapes.h`.

      Also you can create a box polygon with function `SetBox`. Use it when you need rotating boxes.

  - Only a single polygon can rotate.

    - AABB (as it is axis-aligned) and Circles, for sure, don't rotate.
    
    - I didn't implement polygon rotation in the concave (group of triangles).
    
  - function `Transform` in shapes works slightly different in each shapes

    See `SPC_Shapes.h` or examples below.


  - In `test/main_SP2C.cpp`
    - I also implemented some custom shapes, like RoundRect and Concaves.

      - RoundRect is a group of circles and aabbs.

        You can set draw mode to draw each circles and boxes, or outline of roundrect.

        See function `createRoundRect` and `drawRoundRect`.

      - Concave is converted into a group of triangles using Ear Clipping triangulation algorithm.

        Due to the generation code in main function, it can be actually a convex in random cases.

        You can set draw mode to draw each triangles, or outline of concave.

        See function `triangulation` and `drawTriangles`.

      - You can test rotating polygons with the option,

        ```c++
        #define ROTATE_POLYGON_TEST 1 //line 15 in test/main_SP2C.cpp
        ```

      - You can move a shape with arrow keys to test collisions



- Others

  - used [SFML Library](https://www.sfml-dev.org/) to render graphics.

  - environment : C++17, Visual Studio 2019, SFML 2.5.1

  - Please let me know if there is a problem or something to fix.



### Example


```c++
#include <SP2C/SPC_Shapes.h>
#include <SP2C/SPC_Collision.h>

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
    case SP2C::SPC_Shape::AABB:
        reinterpret_cast<SP2C::SPC_AABB*>(m.A)->Translate(f.x, f.y);
        break;
    
    case SP2C::SPC_Shape::Circle:
        reinterpret_cast<SP2C::SPC_Circle*>(m.A)->position += f;
        break;
    
    case SP2C::SPC_Shape::Polygon:
        reinterpret_cast<SP2C::SPC_Polygon*>(m.A)->Translate(f.x, f.y);
        break;
        
    default:
        break;
}
```

The result would be like:

![image_3](https://user-images.githubusercontent.com/26345945/159725513-9bc72251-dcec-4919-b6fa-443b3edcb41b.gif)



You can use function `Translate`, `Scale`, `Rotate` (rotate only in polygon) in each shapes.

Also you can use `Transform` to translae, scale and rotate the shape.

If xscale and yscale are different (`m[0][0] != m[1][1]`), circle will scale `max(m[0][0], m[1][1])`.

You can see how the shape changes with scale matrix below:

```c++
SP2C::SPC_Mat33 matrix = SP2C::SPC_MAT_IDENTITY;
matrix.m[0][0] = 0.5; //xscale
matrix.m[1][1] = 1; //yscale

//center: (300, 300), size: (100, 80)
SP2C::SPC_AABB aabb;
aabb.SetBox(100, 80);
aabb.Translate(300, 300);
aabb.Transform(matrix);

SP2C::SPC_Circle circle;
circle.radius = 50;
circle.position = Vec2(300, 300);

SP2C::SPC_Shape* aabb2 = aabb.Clone();
aabb2->Translate(200, 0);
aabb2->Transform(matrix);

SP2C::SPC_Shape* circle2 = circle.Clone();
circle2->Translate(200, 0);
circle2->Transform(matrix);
```

![Screenshot_5](https://user-images.githubusercontent.com/26345945/160278907-6f26a9e1-9b82-4309-b936-8b00962ec705.png)

AABB decreases in width, but Circle doesn't. (`scale = max(0.5, 1) = 1`)



Scale matrix example 2:

```
SP2C::SPC_Mat33 matrix = SP2C::SPC_MAT_IDENTITY;
matrix.m[0][0] = 0.5; //xscale
matrix.m[1][1] = 1.5; //yscale

//center: (300, 300), size: (100, 80)
SP2C::SPC_AABB aabb;
aabb.SetBox(100, 80);
aabb.Translate(300, 300);
aabb.Transform(matrix);

SP2C::SPC_Circle circle;
circle.radius = 50;
circle.position = Vec2(300, 300);

SP2C::SPC_Shape* aabb2 = aabb.Clone(); //Clone returns SPC_Shape pointer.
aabb2->Translate(200, 0);
aabb2->Transform(matrix);

SP2C::SPC_Shape* circle2 = circle.Clone();
circle2->Translate(200, 0);
circle2->Transform(matrix);
```

![Screenshot_6](https://user-images.githubusercontent.com/26345945/160279101-dcb4ee7d-251e-4047-adb1-3a79d1f62829.png)

AABB decreases in width and increases in height. Circle increases size. (`scale = max(0.5, 1.5) = 1.5`)



As AABB and Circle can not rotate, it works as:

- AABB becomes boundary area of the rotated rectangle.
- Circle does not change anything.

You can see how the shape changes with rotation matrix below:

```c++
SP2C::SPC_Mat33 matrix = SP2C::SPC_MAT_IDENTITY;
matrix.Rotate(30); //rotate +30 (degree)

//center: (300, 300), size: (100, 80)
SP2C::SPC_AABB aabb;
aabb.SetBox(100, 80);
aabb.Translate(300, 300);
aabb.Transform(matrix);

SP2C::SPC_Polygon polygon;
polygon.SetBox(100, 80);
polygon.Translate(300, 300);
polygon.Transform(matrix);
```

![Screenshot_4](https://user-images.githubusercontent.com/26345945/160279143-4da8fd7e-6886-4394-94aa-31406601f51f.png)



The image below shows how AABB and Polygon changes by rotation matrix.

![rotation](https://user-images.githubusercontent.com/26345945/160279150-12b46141-92ce-414c-b3a0-b78f0196ee49.gif)

