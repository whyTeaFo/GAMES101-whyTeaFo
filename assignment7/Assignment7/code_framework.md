# GAMES101 Assignment 7 — 代码框架分析

> 本文梳理 assignment7 的代码框架，帮助理解渲染管线，为在 `Scene.cpp::castRay` 中实现
> Path Tracing 做准备。核心结论：**只需要在 `Scene.cpp` 的 `castRay` 中写一个函数**，
> 其余求交 / BVH / 材质 / 光源采样 / 主渲染循环均由框架提供。

---

## 1. 项目概览

这是一个基于 CPU 的**路径追踪（Path Tracing）渲染器**，渲染经典的 Cornell Box 场景。

整体是典型的「离线渲染管线」：

```
main.cpp 构建场景 → Scene::buildBVH 建加速结构 → Renderer::Render 逐像素发射主光线
→ Scene::castRay 递归求 radiance（本次要实现的函数）→ 输出 binary.ppm
```

- 相机：`eye_pos = (278, 273, -800)`，朝 +z 方向看，fov = 40。
- 分辨率：`Scene scene(784, 784)`，每像素 `spp = 16` 次采样（均无抖动，重复采样降方差）。
- 场景物体：6 个 `MeshTriangle`（地面、矮箱、高箱、左红墙、右绿墙、天花板上的面光源）。

---

## 2. 文件清单与职责

| 文件 | 职责 |
|---|---|
| `main.cpp` | 程序入口。创建 Scene、定义 4 种材质（red / green / white / light）、用 OBJ_Loader 加载 6 个网格、`buildBVH()`、调用 `Renderer::Render` 并计时 |
| `Renderer.cpp / .hpp` | 主渲染循环：算每个像素的主光线方向，循环 spp 次调用 `castRay` 并累加平均；最后写 PPM 文件（含 gamma 0.6 校正）。**定义全局 `EPSILON = 1e-5`** |
| `Scene.hpp / .cpp` | 场景容器（objects、lights、相机/渲染参数）。**`castRay` 为空、待实现**；`sampleLight` 按面积采样发光物体；`intersect` 走 BVH；`trace` 是朴素线性求交（本作业不用） |
| `BVH.hpp / .cpp` | BVH 加速结构：NAIVE 递归建树、`getIntersection` 遍历求交、`Sample` 按三角形面积加权采样（供光源采样使用） |
| `Bounds3.hpp` | AABB 包围盒。`IntersectP` 用 slab 法 + `direction_inv` 加速求交 |
| `Object.hpp` | 抽象基类：`intersect / getIntersection / getSurfaceProperties / getArea / Sample / hasEmit` 等纯虚接口 |
| `Triangle.hpp` | `Triangle` 单个三角形：Möller–Trumbore 求交、均匀面采样（`pdf=1/area`）、`MeshTriangle` 网格（内部再挂一个 BVH，委托求交与采样） |
| `Sphere.hpp` | 球体求交 / 采样（本场景未使用） |
| `Material.hpp` | 材质，仅 `DIFFUSE`：`sample`（半球均匀采样）、`pdf`（= 1/2π）、`eval`（= Kd/π）、`hasEmission` |
| `Light.hpp / AreaLight.hpp` | 点光源基类 / 面光源类。**本场景实际不用它们**：光源是 light.obj 这个 `MeshTriangle` + 发光材质 |
| `Ray.hpp` | 光线结构（origin / direction / direction_inv / t_min / t_max） |
| `Intersection.hpp` | 交点信息结构体 |
| `global.hpp` | `M_PI`、`EPSILON`（extern）、`kInfinity`、`solveQuadratic`、`get_random_float`、`UpdateProgress` 进度条 |
| `Vector.hpp / Vector.cpp` | `Vector3f / Vector2f` 及点积、叉积、归一化等运算 |
| `OBJ_Loader.hpp` | OBJ 模型解析器 |
| `CMakeLists.txt` | CMake 构建（C++17，`add_executable(RayTracing ...)`） |

---

## 3. 核心数据结构

### 3.1 Intersection（交点）
```cpp
struct Intersection {
    bool    happened;   // 是否命中
    Vector3f coords;    // 命中点坐标
    Vector3f normal;    // 命中点法线
    Vector3f emit;      // 自发光（仅光源采样时由 Sample 填充）
    double  distance;   // 命中点到光线原点的距离
    Object* obj;        // 命中的物体指针
    Material* m;        // 命中点的材质
};
```

### 3.2 Ray（光线）
```cpp
struct Ray {
    Vector3f origin;
    Vector3f direction, direction_inv; // direction_inv 供 BVH slab 测试加速
    double t, t_min, t_max;
    Vector3f operator()(double t) const { return origin + direction * t; }
};
```

### 3.3 Material（DIFFUSE 三个关键函数）
- `sample(wi, N)`：以 N 为 +z 构造局部系，对半球做**均匀采样**再 `toWorld` 转回世界系。
  保证 `dot(返回方向, N) >= 0`。
- `pdf(wi, wo, N)`：`dot(wo, N) > 0` 时返回 `0.5/π`（均匀半球采样密度），否则返回 0。
- `eval(wi, wo, N)`：`dot(N, wo) > 0` 时返回 `Kd / π`（Lambert 漫反射），否则返回 0。
- `hasEmission()`：`m_emission.norm() > EPSILON`。

> 注意函数签名里的参数名有误导性：`eval(wi, wo, N)` 实际是用**第二个参数**（wo）与 N 做点积、
> 并作为出射方向参与 BRDF。调用时要写 `m->eval(ray.direction, lightDir, N)`。

### 3.4 Scene（场景）
```cpp
class Scene {
    int width = 1280, height = 960;   // main.cpp 中改为 784x784
    double fov = 40;
    int maxDepth = 1;                 // 本作业不靠 maxDepth 截断，靠俄罗斯轮盘赌
    float RussianRoulette = 0.8;
    std::vector<Object*> objects;
    std::vector<std::unique_ptr<Light>> lights;  // 空！光源就是 objects 里的 light_
    BVHAccel *bvh;
    Intersection intersect(const Ray&) const;   // BVH 求交
    void sampleLight(Intersection& pos, float& pdf) const; // 光源面积加权采样
    Vector3f castRay(const Ray&, int depth) const; // ★ 待实现
};
```

---

## 4. 渲染流程

### 4.1 main.cpp
```cpp
Scene scene(784, 784);
// 4 个材质：red/green/white(Kd 不同)，light(m_emission 很大)
MeshTriangle floor / shortbox / tallbox / left / right / light_(...);
scene.Add(...) × 6;
scene.buildBVH();          // BVHAccel(objects, 1, SplitMethod::NAIVE)
Renderer r; r.Render(scene);
```

### 4.2 Renderer::Render
```cpp
float scale = tan(deg2rad(fov * 0.5));       // 透视投影
for j in [0,height): for i in [0,width):
    x = (2*(i+0.5)/width - 1) * imageAspectRatio * scale;
    y = (1 - 2*(j+0.5)/height) * scale;
    dir = normalize(Vector3f(-x, y, 1));     // 主光线方向（相机看向 +z）
    for k in [0, spp):                       // spp = 16
        framebuffer[m] += castRay(Ray(eye_pos, dir), 0) / spp;
// 写 binary.ppm：P6 + 每像素 RGB，clamp 后 pow(c, 0.6) 伽马校正
```

要点：每个像素 **spp 条光线走同一个方向**，castRay 内部用随机数做光源采样和半球采样，
所以重复采样能降噪。调试时把 `spp` 改成 1、把分辨率改成 `(128,128)` 即可快速出图。

---

## 5. 求交与 BVH

- `Scene::buildBVH()` → `BVHAccel(objects, 1, NAIVE)`，根节点存所有物体，递归按
  **centroid 最大轴**排序后二分，叶子节点 = 单个 `Object`。
- `Scene::intersect(ray)` → `bvh->Intersect(ray)` → 递归 `getIntersection(node, ray)`：
  1. `Bounds3::IntersectP`（slab 法，用 `ray.direction_inv` 和方向正负号数组加速）快速拒绝；
  2. 叶子节点调用 `node->object->getIntersection(ray)`；
  3. 内部节点取左右子树中 `distance` 更近的那个。
- `Triangle::getIntersection`：Möller–Trumbore 求交。**带背面剔除**：
  `if (dot(ray.direction, normal) > 0) return 未命中;`。所以面光源只有朝下的一面能被「从下方」
  的光线打中。
- `MeshTriangle` 内部为每个三角形建了一个 BVH，`getIntersection` 委托给该 BVH。
- `Scene::trace` 是纯线性扫描求交（框架遗留，本作业不使用）。

---

## 6. 光源与采样

- 光源 = light.obj 加载出的 `MeshTriangle`，`light` 材质的 `m_emission` 数值很大（约 8~18 量级），
  几何上是 y=548.7 处一块 130×105 的平面，法线朝下（见 3.5 节推论）。
- `Scene::sampleLight(pos, pdf)`：
  ```cpp
  emit_area_sum = Σ hasEmit() ? getArea() : 0;
  p = get_random_float() * emit_area_sum;      // p ∈ [0, total)
  for 每个发光物体: 累加面积, 若 p <= 累加值 → 该物体 Sample(pos, pdf);
  ```
  → 按面积加权的**均匀采样**。
- `MeshTriangle::Sample(pos, pdf)`：委托内部 BVH 的 `Sample`（沿面积加权下探到叶子三角形，
  `Triangle::Sample` 给 `pdf = 1/tri.area`，再乘/除面积最后得到 `pdf = 1/光源总面积`），
  并填充 `pos.emit = m->getEmission()`。
- ⚠️ 坑：`sampleLight` 返回的 pdf 是**被选中物体**的 `1/area`，若场景有多个发光物体，
  还应乘上选中该物体的概率（面积占比），否则 pdf 有误。本场景只有一个光源，无影响。

---

## 7. castRay 需要实现的 Path Tracing（算法与代码对应）

```
shade(p, wo)  →  castRay(Ray(p, wo), depth)
  ① p_ = intersect(p, wo)                      → Scene::intersect
     若未命中 → return 0
     若 p_.m->hasEmission() → return m->getEmission()   // ★ 关键：让光源可见
  ② 直接光照（对光源均匀采样 1 次）
     sampleLight(x_, pdf_light)                → Scene::sampleLight
     L_dir = L_i * f_r * cosθ / |x'-p|² / pdf_light
           = x_.emit * m->eval(wo, lightDir, N) * dot(lightDir, N)
             / lightDist² / lightPdf
     阴影判定：从 p + N*EPSILON 沿 lightDir 发射 shadowRay
       若最近交点距离 ≥ |x'-p| - EPSILON → 未被遮挡，才累加 L_dir
  ③ 间接光照（俄罗斯轮盘赌 + 半球采样）
     if rand < RussianRoulette:
        wi = m->sample(wo, N)                  → Material::sample
        从 p + N*EPSILON 沿 wi 发射 bounceRay
        若命中且非发光:  L_indir = castRay(bounceRay, depth+1)
                                * m->eval(wo, wi, N) * dot(wi, N)
                                / m->pdf(wo, wi, N) / RussianRoulette
  ④ return L_dir + L_indir
```

相关常量：`EPSILON = 1e-5`（Renderer.cpp 定义，global.hpp 声明 extern）、
`M_PI`、`RussianRoulette = 0.8`。

---

## 8. 数值精度与常见坑（调试必读）

1. **自相交 / 阴影偏移**：阴影光线和反射光线的起点必须沿法线偏移 `N * EPSILON`，
   否则光线会在命中点处再次击中同一平面。
2. **阴影边界判定**：比较 `阴影交点距离` 与 `到光源距离` 时用 `> -EPSILON` 的容差，
   不要用 `>`。浮点误差会把「恰好贴着光源」的点误判为被遮挡，产生黑色斑点 / **黑色横条纹**。
3. **pdf 接近 0**：除法前应对 `pdf` 做防御（`pdf > EPSILON`）。`Material::pdf` 在
   `dot(wo,N) <= 0` 时返回 0，若被除数会得到 inf/NaN → 噪点。半球采样 `sample` 保证
   `dot(wi,N) >= 0`，所以间接光照里 pdf 恒为 0.5/π，一般安全。
4. **光源纯黑现象**：如果 castRay 一开头不处理 `hasEmission()`，主光线直接打到光源表面时，
   shade() 只会计算该点的「反射」radiance：直接光照由于对光源表面上的点再采样光源，
   `dot(lightDir, N)≈0` 且阴影光线会被光源自身遮挡 → `L_dir≈0`，于是光源只剩极弱的
   间接反射 → 表现为纯黑。**修复：命中发光表面时直接 `return m->getEmission()`。**

---

## 9. 调试加速建议

- 渲染复杂度 ≈ O(像素 × spp × 递归深度 × BVH 求交)。默认 784×784×16 很慢。
  - 调分辨率：`Scene scene(784, 784)` → `(128, 128)`，速度快 ~37 倍。
  - 调采样：`Renderer.cpp` 中 `int spp = 16` → `1`，再快 16 倍。
- **多线程**：`Renderer::Render` 的像素循环可并行化（每行或每块一个线程）。注意：
  `get_random_float()` 每次调用都 new 一个 `std::random_device`+`std::mt19937`，
  不仅慢，且多线程下若共用同一引擎会有数据竞争；应改成**线程局部**的静态随机引擎。
- 把 `get_random_float` 换成线程局部引擎本身就能带来可观加速（避免反复初始化随机设备）。
