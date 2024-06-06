import findBaseLink from "../../utils/findBaseLink";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useRef, useState } from "react";

// RecursiveTreeView Component
export const LinkTree = ({ scene, selectObject, selectedObject, deleteObject, duplicateObject, getBaseLink }) => {
    const objectContextMenu = useRef(null);
    const [objectContextMenuPosition, setUseObjectContextMenuPosition] = useState({ left: -1000, top: -10000 });
    const [lastButtonObjectSelected, setLastButtonObjectSelected] = useState(null);

    // this keeps the context menu from coming back on when a previously right clicked object that was unselected is selected again
    if (lastButtonObjectSelected !== selectedObject && lastButtonObjectSelected) {
        setLastButtonObjectSelected(null);
    }

    // Function to render each node and its children
    const renderNode = (node) => {
        if (!node) {
            return null;
        }
        // Display the current node's data and recursively render its children
        return (
            <div style={{ marginLeft: "20px" }}>
                {node.userData && (
                    <button
                        className="tree-item"
                        onClick={() => {
                            selectObject(node);
                        }}
                        onContextMenu={(e) => {
                            e.preventDefault();
                            if (node.userData.isBaseLink) {
                                setLastButtonObjectSelected(null);
                                return;
                            }
                            selectObject(node);
                            setLastButtonObjectSelected(node);
                            setUseObjectContextMenuPosition({ left: e.clientX, top: e.clientY });
                        }}
                    >
                        {node.userData.name}
                    </button>
                )}
                {node.getChildren() && node.getChildren().length > 0 && (
                    <div>
                        {node
                            .getChildren()
                            .filter((child) => child.urdfObject)
                            .map((child) => renderNode(child))}
                    </div>
                )}
            </div>
        );
    };

    // let node = findBaseLink(scene);
    let node = getBaseLink();

    const hideContextMenu = () => {
        setLastButtonObjectSelected(null);
    };

    return (
        <div className="object-tree" onClick={hideContextMenu} onMouseLeave={hideContextMenu}>
            Object Tree
            <div className="scroll-box">{node && renderNode(node)}</div>
            {lastButtonObjectSelected === selectedObject && selectedObject && (
                <ObjectContextMenu
                    objectContextMenu={objectContextMenu}
                    objectContextMenuPosition={objectContextMenuPosition}
                    deleteObject={deleteObject}
                    duplicateObject={duplicateObject}
                    selectedObject={selectedObject}
                />
            )}
        </div>
    );
};
