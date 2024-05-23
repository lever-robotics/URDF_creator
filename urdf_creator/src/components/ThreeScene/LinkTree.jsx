import { ObjectContextMenu } from '../../utils/ObjectContextMenu';
import React, { useRef, useState } from "react";

// RecursiveTreeView Component
export const LinkTree = ({ tree, select, updateTree, selectedObject, setSelectedObject, transformControls }) => {

    const objectContextMenu = useRef(null);
    const [objectContextMenuPosition, setUseObjectContextMenuPosition] = useState({ left: -1000, top: -10000 })
    const [lastButtonObjectSelected, setLastButtonObjectSelected] = useState(null);

    // this keeps the context menu from coming back on when a previously right clicked object that was unselected is selected again
    if (lastButtonObjectSelected !== selectedObject && lastButtonObjectSelected) {
        setLastButtonObjectSelected(null)
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
                            select(node);
                        }}
                        onContextMenu={(e) => {
                            e.preventDefault();
                            if (node.userData.isBaseLink) {
                                setLastButtonObjectSelected(null)
                                return
                            };
                            select(node)
                            setSelectedObject(node)
                            setLastButtonObjectSelected(node)
                            setUseObjectContextMenuPosition({ left: e.clientX, top: e.clientY })
                        }}
                    >
                        {node.userData.name}
                    </button>
                )}
                {node.userData.scaler.children && node.userData.scaler.children.length > 0 && (
                    <div>{node.userData.scaler.children.filter((child) => child.type === "Mesh").map((child) => renderNode(child))}</div>
                )}
            </div>
        );
    };

    let node = null;
    if (tree) {
        if (tree.children) {
            const children = tree.children.filter((child) => child.type === "Mesh");
            if (children.length > 0) {
                node = children[0];
            }
        }
    }

    const hideContextMenu = () => { setLastButtonObjectSelected(null) }

    return (
        <div className="object-tree" onClick={hideContextMenu} onMouseLeave={hideContextMenu}>
            Object Tree
            <div className="scroll-box">{node && renderNode(node)}</div>
            {(lastButtonObjectSelected === selectedObject) && selectedObject && <ObjectContextMenu objectContextMenu={objectContextMenu} objectContextMenuPosition={objectContextMenuPosition} selectedObject={selectedObject} updateTree={updateTree} setSelectedObject={setSelectedObject} transformControls={transformControls} />}
        </div>
    );
};
