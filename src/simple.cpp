#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList &props) {
        /* No parameters this time */
        position = props.getPoint("position");
        phi = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {


        /* Find the surface that is visible in the requested direction */
        Intersection its, its_;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading simple as a color */
        auto y = (position-its.p).dot(its.shFrame.n)/
            std::sqrt(its.shFrame.n.squaredNorm()*(its.p-position).squaredNorm()); // computest cosine using dot products

        y = std::max((float)0,y); //taking the max value
        Ray3f ray_(its.p, position - its.p);
        float v = y;
        if (scene->rayIntersect(ray_, its_, true))
            v = 0;
        auto s = v * phi / ( (its.p-position).squaredNorm() * 4 * M_PI * M_PI);
                    
        Normal3f n = its.shFrame.n.cwiseAbs();        
        return Color3f( s.x(),  s.y() ,  s.z());
    }

    std::string toString() const {
        return "simpleIntegrator[]";
    }
protected:
    Point3f position;
    Color3f phi;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");


NORI_NAMESPACE_END

