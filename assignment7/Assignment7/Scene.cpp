//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // (1) 与场景求交
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f(0.0f);
    }

    // (2) 直接命中发光表面（光源）：返回其自发光，否则光源会被渲染成纯黑
    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    const Vector3f N = inter.normal;
    const Vector3f wo = ray.direction;

    // ==================== 直接光照：对光源均匀采样一次 ====================
    Vector3f L_dir(0.0f);
    Intersection lightInter;
    float lightPdf = 0.0f;
    sampleLight(lightInter, lightPdf);

    if (lightPdf > EPSILON) {
        Vector3f lightDir = lightInter.coords - inter.coords;   // light to inter
        float lightDist = std::sqrt(dotProduct(lightDir, lightDir));
        float lightDist2 = lightDist * lightDist;
        lightDir = normalize(lightDir);

        // 阴影判定：从 p 沿光源方向发一条光线，若最近交点不比光源更近则未被遮挡
        // （用 -EPSILON 容差吸收浮点误差，避免贴光源的表面被误判为遮挡 → 黑斑/黑横纹）
        Ray shadowRay(inter.coords + N * EPSILON, lightDir);
        Intersection shadowInter = intersect(shadowRay);
        if (shadowInter.happened && shadowInter.distance - lightDist > -EPSILON) {
            L_dir = lightInter.emit * inter.m->eval(wo, lightDir, N)
                    * dotProduct(lightDir, N) / lightDist2 / lightPdf;
        }
    }

    // ==================== 间接光照：俄罗斯轮盘赌 + 半球均匀采样 ====================
    Vector3f L_indir(0.0f);
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, N);
        Ray bounceRay(inter.coords + N * EPSILON, wi);
        Intersection bounce = intersect(bounceRay);
        // 只对非发光表面递归，发光表面由直接光照项（或上面的 emission 分支）贡献
        if (bounce.happened && !bounce.m->hasEmission()) {
            L_indir = castRay(bounceRay, depth + 1)
                    * inter.m->eval(wo, wi, N)
                    * dotProduct(wi, N)
                    / inter.m->pdf(wo, wi, N)
                    / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}