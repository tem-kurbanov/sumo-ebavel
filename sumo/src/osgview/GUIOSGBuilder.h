/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    GUIOSGBuilder.h
/// @author  Daniel Krajzewicz
/// @date    19.01.2012
///
// Builds OSG nodes from microsim objects
/****************************************************************************/
#pragma once
#include <config.h>

#ifdef HAVE_OSG

#include "GUIOSGView.h"

#include <map>


// ===========================================================================
// class declarations
// ===========================================================================

namespace osg {
class Node;
class Group;
class PositionAttitudeTransform;
}

namespace osgUtil {
class Tessellator;
}

class MSVehicleType;
class MSEdge;
class GUIJunctionWrapper;

namespace std {
    template <>
    struct hash<osg::Vec4ub> {
        std::size_t operator()(const osg::Vec4ub& v) const noexcept {
            return (static_cast<std::size_t>(v.r()) << 24) |
                    (static_cast<std::size_t>(v.g()) << 16) |
                    (static_cast<std::size_t>(v.b()) << 8) |
                    static_cast<std::size_t>(v.a());
        }
    };
}

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIOSGBuilder
 * @brief Builds OSG nodes from microsim objects
 */
class GUIOSGBuilder {
public:
    static osg::Group* buildOSGScene(osg::Node* const tlg, osg::Node* const tly, osg::Node* const tlr, osg::Node* const tlu, osg::Node* const pole, osg::
                                     MatrixTransform *plane);

    static void buildDecal(const GUISUMOAbstractView::Decal& d, osg::Group& addTo);

    static GUIOSGView::OSGLight* buildLight(const GUISUMOAbstractView::Decal& d, osg::Group& addTo);
    static void updateLight(const GUISUMOAbstractView::Decal& d, GUIOSGView::OSGLight* light);

    /// @brief Build traffic light models with poles and cantilevers automatically
    static void buildTrafficLightDetails(MSTLLogicControl::TLSLogicVariants& vars, osg::Node* const tlg, osg::Node* const tly, osg::Node* const tlr, osg::Node* const tlu, osg::Node* poleBase, osg::Group& addTo);

    static osg::PositionAttitudeTransform* getTrafficLight(const GUISUMOAbstractView::Decal& d, MSTLLogicControl::TLSLogicVariants& vars, const MSLink* link, osg::Node* const tlg,
            osg::Node* const tly, osg::Node* const tlr, osg::Node* const tlu, osg::Node* const pole, const bool withPole = false, const double size = -1, double poleHeight = 1.8, double transparency = .3);

    static GUIOSGView::OSGMovable buildMovable(const MSVehicleType& type);

    /// @brief set vehicle model body color
    static void setVehBodyColor(GUIOSGView::OSGMovable& m, osg::Vec4ub color);

    static osg::Node* buildSkybox(osg::Image* px, osg::Image* nx, osg::Image* py, osg::Image* ny, osg::Image* pz, osg::Image* nz);

    static osg::Node* buildPlane(const float baseZ = -0.1f); // OSG needs float coordinates here

    static void updateDecalTransform(const GUISUMOAbstractView::Decal& d);
private:
    static osg::PositionAttitudeTransform* createTrafficLightState(const GUISUMOAbstractView::Decal& d, osg::Node* tl, const double withPole, const double size, osg::Vec4d color);

    static void buildOSGEdgeGeometry(const MSEdge& edge,
                                     osg::Group& addTo, osgUtil::Tessellator& tessellator);

    static void buildOSGJunctionGeometry(GUIJunctionWrapper& junction,
                                         osg::Group& addTo, osgUtil::Tessellator& tessellator);

    static void buildOSGPolygonGeometry(const SUMOPolygon& polygon,
                                        osg::Group& addTo, osgUtil::Tessellator& tessellator, float height);

    static void setShapeState(osg::ref_ptr<osg::ShapeDrawable> shape);

    static std::map<std::string, osg::ref_ptr<osg::Node> > myCars;
    static std::map<std::string, std::map<osg::Vec4ub, osg::ref_ptr<osg::Node>>> myColoredCars;
    static std::map<std::string, std::map<std::string, osg::ref_ptr<osg::Node>>> myCarsParts;
    static std::map<std::string, std::unordered_map<std::string, osg::ref_ptr<osg::Material>>> myCarsMaterials;
    static std::map<std::string, osg::ref_ptr<osg::PositionAttitudeTransform> > myLoadedDecalTransforms;
    static std::map<std::string, osg::ref_ptr<osg::Node> > myLoadedDecals;
};

/**
 * @class SkyBox
 * @brief Implements skybox object.
 */
class SkyBox : public osg::Transform {
public:
    SkyBox();
    SkyBox( const SkyBox& copy, osg::CopyOp copyop = osg::CopyOp::SHALLOW_COPY ) : osg::Transform(copy, copyop) {}

    void setEnvironmentMap(osg::Image* px, osg::Image* nx, osg::Image* py, osg::Image* ny, osg::Image* pz, osg::Image* nz);

    bool computeLocalToWorldMatrix(osg::Matrix &matrix, osg::NodeVisitor *) const override;
    bool computeWorldToLocalMatrix(osg::Matrix &matrix, osg::NodeVisitor *) const override;
protected:
    virtual ~SkyBox() = default;
};


#endif
