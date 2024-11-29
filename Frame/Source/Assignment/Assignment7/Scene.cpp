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
    // Implement Path Tracing Algorithm here
    Vector3f result = { 0, 0, 0 };

    // Find the intersection point of the ray
    Intersection inter = Scene::intersect(ray);

    // Get the shade value by path tracing
    if (inter.happened) {
        result = pathTraceShade(inter, -ray.direction);
    }

    return result;
}

Vector3f Scene::pathTraceShade(const Intersection& shadeInter, const Vector3f& outDir) const
{
    /*
     * The Path Tracing Algorithm.
     * \param shadeInter: Intersection struct returned by Scene::intersect
     * \param outDir: normailized direction from shade point to eye
     */
    Vector3f directLight = { 0, 0, 0 };
    Vector3f indirectLight = { 0, 0, 0 };

    Intersection samplePos;
    float pdf;
    Vector3f shadePoint = shadeInter.coords;
    Vector3f shadeNormal = shadeInter.normal;
    Material* material = shadeInter.m;
    
    if (!material) return Vector3f{};

    /// Step 0. Add the energy if the shade point itself is a light
    if (material->hasEmission()) {
        directLight = material->getEmission();
    }

    /// Step 1. Calculate shade value directly from light (By sampling light).
    // Sample the light uniformly
    sampleLight(samplePos, pdf);
    if (samplePos.emit.norm() > EPSILON) {    // Light mush have emission
        // Check wheather the sampling light ray is not blocked by other objects
        Vector3f point2LightDir = (samplePos.coords - shadePoint).normalized();
        Ray point2Light(shadePoint, point2LightDir);
        Intersection hitPos = Scene::intersect(point2Light);
        if (hitPos.happened && fabs(hitPos.coords.x - samplePos.coords.x) < EPSILON
            && fabs(hitPos.coords.y - samplePos.coords.y) < EPSILON
            && fabs(hitPos.coords.z - samplePos.coords.z) < EPSILON) {
            
            Vector3f lightNormal = samplePos.normal;
            Vector3f inDir = -point2LightDir;       // inDir: from light to shade point
            directLight += samplePos.emit * material->eval(-inDir, outDir, shadeNormal)
                * (dotProduct(-inDir, shadeNormal) * dotProduct(inDir, lightNormal)
                / (powf((samplePos.coords - shadePoint).norm(), 2)) / pdf);
        }
    }

    /// Step 2. Calculate shade value indirectly from reflection (By sampling in_dir).
    // Test the Russia Roulette
    float rand = get_random_float();
    if (rand <= RussianRoulette) {
        // Sample the in_dir uniformly
        Vector3f inDir = material->sample(outDir, shadeNormal);   // inDir: from shade point to next hit point
        // Check wheather the inDir ray hits a non-emitting object
        Ray inDirRay(shadePoint, inDir);
        Intersection hitPos = Scene::intersect(inDirRay);
        if (hitPos.happened && hitPos.m && !hitPos.m->hasEmission()) {
            indirectLight = pathTraceShade(hitPos, -inDir) * material->eval(inDir, outDir, shadeNormal)
                * dotProduct(inDir, shadeNormal) / material->pdf(inDir, outDir, shadeNormal) / RussianRoulette;
        }
    }

    return directLight + indirectLight;
}