//
//  PropGroup.qml
//
//  Created by Sam Gateau on 3/2/2019
//  Copyright 2019 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or https://www.apache.org/licenses/LICENSE-2.0.html
//

import QtQuick 2.7

// PropGroup is mostly reusing the look and feel of the PropFolderPanel
// It is populated by calling "updatePropItems"
// or adding manually new Items to the "propItemsPanel"
PropFolderPanel {    
    Global { id: global }
    id: root
    
    property alias propItemsPanel: root.panelFrameContent
    
    // Prop Group is designed to author an array of ProItems, they are defined with an array of the tuplets describing each individual item:
    // [ ..., PropItemInfo, ...]
    // PropItemInfo {
    //    type: "PropXXXX", object: JSobject, property: "propName"      
    // }
    //
    function updatePropItems(propItemsContainer, propItemsModel) {
        root.hasContent = false
        for (var i = 0; i < propItemsModel.length; i++) {
            var proItem = propItemsModel[i];
            // valid object
            if (proItem['object'] !== undefined && proItem['object'] !== null ) {
                // valid property
                if (proItem['property'] !== undefined && proItem.object[proItem.property] !== undefined) {
                    // check type
                    if (proItem['type'] === undefined) {
                        proItem['type'] = typeof(proItem.object[proItem.property])
                    }
                    switch(proItem.type) {
                        case 'boolean':
                        case 'PropBool': {
                            var component = Qt.createComponent("PropBool.qml");
                            component.createObject(propItemsContainer, {
                                "label": proItem.property,
                                "object": proItem.object,
                                "property": proItem.property
                            })
                        } break;
                        case 'number':
                        case 'PropScalar': {
                            var component = Qt.createComponent("PropScalar.qml");
                            component.createObject(propItemsContainer, {
                                "label": proItem.property,
                                "object": proItem.object,
                                "property": proItem.property,
                                "min": (proItem["min"] !== undefined ? proItem.min : 0.0),                   
                                "max": (proItem["max"] !== undefined ? proItem.max : 1.0),                                       
                                "integer": (proItem["integral"] !== undefined ? proItem.integral : false),
                            })
                        } break;
                        case 'PropEnum': {
                            var component = Qt.createComponent("PropEnum.qml");
                            component.createObject(propItemsContainer, {
                                "label": proItem.property,
                                "object": proItem.object,
                                "property": proItem.property,
                                "enums": (proItem["enums"] !== undefined ? proItem.enums : ["Undefined Enums !!!"]), 
                            })
                        } break;
                        case 'object': {
                            var component = Qt.createComponent("PropItem.qml");
                            component.createObject(propItemsContainer, {
                                "label": proItem.property,
                                "object": proItem.object,
                                "property": proItem.property,
                             })
                        } break;
                        case 'printLabel': {
                            var component = Qt.createComponent("PropItem.qml");
                            component.createObject(propItemsContainer, {
                                "label": proItem.property
                             })
                        } break;
                    }
                    root.hasContent = true
                } else {
                    console.log('Invalid property: ' + JSON.stringify(proItem));
                }
            } else if (proItem['type'] === 'printLabel') {
                var component = Qt.createComponent("PropItem.qml");
                component.createObject(propItemsContainer, {
                    "label": proItem.label
                })     
            } else {
                console.log('Invalid object: ' + JSON.stringify(proItem));
            }
        }
    }
    Component.onCompleted: {
    }
}
