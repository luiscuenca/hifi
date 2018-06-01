//
//  OverlaysPayload.cpp
//  interface/src/ui/overlays
//
//  Copyright 2014 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include <limits>
#include <typeinfo>

#include <avatar/AvatarManager.h>
#include <avatar/MyAvatar.h>
#include <LODManager.h>
#include <render/Scene.h>

#include "Image3DOverlay.h"
#include "Circle3DOverlay.h"
#include "Cube3DOverlay.h"
#include "ImageOverlay.h"
#include "Line3DOverlay.h"
#include "ModelOverlay.h"
#include "Overlays.h"
#include "Rectangle3DOverlay.h"
#include "Sphere3DOverlay.h"
#include "Grid3DOverlay.h"
#include "TextOverlay.h"
#include "Text3DOverlay.h"


namespace render {
    template <> const ItemKey payloadGetKey(const Overlay::Pointer& overlay) {
        auto builder = ItemKey::Builder().withTypeShape();
        if (overlay->is3D()) {
            auto overlay3D = std::static_pointer_cast<Base3DOverlay>(overlay);
            if (overlay3D->getDrawInFront()) {
                builder.withLayer(render::hifi::LAYER_3D_FRONT);
            } else if (overlay3D->getDrawHUDLayer()) {
                builder.withLayer(render::hifi::LAYER_3D_HUD);
            }

            if (overlay->isTransparent()) {
                builder.withTransparent();
            }
        } else {
            builder.withViewSpace();
            builder.withLayer(render::hifi::LAYER_2D);
        }

        if (!overlay->getVisible()) {
            builder.withInvisible();
        }

        // always visible in primary view.  if isVisibleInSecondaryCamera, also draw in secondary view
        uint32_t viewTagBits = render::hifi::TAG_MAIN_VIEW |
            (overlay->getIsVisibleInSecondaryCamera() ? render::hifi::TAG_SECONDARY_VIEW : render::hifi::TAG_NONE);

        builder.withTagBits(viewTagBits);

        return builder.build();
    }
    template <> const Item::Bound payloadGetBound(const Overlay::Pointer& overlay) {
        return overlay->getBounds();
    }
    template <> void payloadRender(const Overlay::Pointer& overlay, RenderArgs* args) {
        if (args) {
            overlay->render(args);
        }
    }
    template <> const ShapeKey shapeGetShapeKey(const Overlay::Pointer& overlay) {
        return overlay->getShapeKey();
    }

    template <> uint32_t metaFetchMetaSubItems(const Overlay::Pointer& overlay, ItemIDs& subItems) {
        return overlay->fetchMetaSubItems(subItems);
    }
}

