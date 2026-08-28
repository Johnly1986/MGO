#include "GeoreferencingFactory.h"
#include "GeoreferencingWith7Parameters.h"
#include "GeoreferencingWithAnchor.h"
#include "GeoreferencingWithMultiPosition.h"
#include <iostream>

std::unique_ptr<IGeoreferencing> GeoreferencingFactory::Create(
    GeoreferencingType type,
    const std::string& prjFile,
    const GeoreferencingOptions& opts)
{
    switch (type)
    {
    case GeoreferencingType::None:
        return nullptr;

    case GeoreferencingType::Identity:
    {
        auto* g = new GeoreferencingWith7Parameters(prjFile, opts.targetCrs);
        g->SetParameter(SevenParameter());  // All zeros — skip Helmert entirely
        g->Solve();
        if (!g->InitPROJPipelines()) {
            std::cerr << "[Factory] Identity: InitPROJPipelines failed — "
                      << "InverseTransform unavailable" << std::endl;
        }
        return std::unique_ptr<IGeoreferencing>(g);
    }

    case GeoreferencingType::SevenParam:
    {
        auto* g = new GeoreferencingWith7Parameters(prjFile, opts.targetCrs);
        g->SetParameter(SevenParameter(
            opts.helmert[0], opts.helmert[1], opts.helmert[2],
            opts.helmert[3], opts.helmert[4], opts.helmert[5],
            opts.helmert[6]));
        g->Solve();
        if (!g->InitPROJPipelines()) {
            std::cerr << "[Factory] SevenParam: InitPROJPipelines failed — "
                      << "InverseTransform unavailable" << std::endl;
        }
        return std::unique_ptr<IGeoreferencing>(g);
    }

    case GeoreferencingType::Anchor:
    {
        auto* g = new GeoreferencingWithAnchor(prjFile, opts.targetCrs);
        g->SetParameter(Eigen::Vector3d(opts.anchorX, opts.anchorY, opts.anchorZ));
        g->Solve();
        if (!g->InitPROJPipelines()) {
            std::cerr << "[Factory] Anchor: InitPROJPipelines failed — "
                      << "Transform unavailable" << std::endl;
        }
        return std::unique_ptr<IGeoreferencing>(g);
    }

    case GeoreferencingType::MultiPosition:
    {
        auto* g = new GeoreferencingWithMultiPosition(prjFile, opts.targetCrs);
        // Control points are set externally by caller via SetParameter().
        // Solve() is called by the caller AFTER SetParameter().
        // MultiPosition uses manual math, not PROJ pipelines.
        // InitPROJPipelines is NOT called — the class overrides
        // InverseTransform/TransformTargetToECEF/GCSNormal to avoid
        // dereferencing null PROJ pointers.
        return std::unique_ptr<IGeoreferencing>(g);
    }

    default:
        std::cerr << "[GeoreferencingFactory] Unknown type" << std::endl;
        return nullptr;
    }
}
