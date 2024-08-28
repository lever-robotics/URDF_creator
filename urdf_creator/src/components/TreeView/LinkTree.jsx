import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useState } from "react";

// RecursiveTreeView Component
export function LinkTree({ selectedObject, stateFunctions }) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [contextMenuVisible, setContextMenuVisible] = useState(false);

    const handleContextMenu = (e, node) => {
        e.preventDefault();
        stateFunctions.selectObject(node);
        setContextMenuVisible(true);
        setContextMenuPosition({
            left: e.clientX,
            top: e.clientY,
        });
    };

    const hideContextMenu = () => {
        setContextMenuVisible(false);
    };

    const baseLink = stateFunctions.getBaseLink();

    return (
        <div className="object-tree" onClick={hideContextMenu} onMouseLeave={hideContextMenu}>
            Link Tree
            <div className="scroll-box">{baseLink && <Node node={baseLink} handleContextMenu={handleContextMenu} stateFunctions={stateFunctions} />}</div>
            {contextMenuVisible && (
                <ObjectContextMenu
                    // objectContextMenu={objectContextMenu}
                    contextMenuPosition={contextMenuPosition}
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            )}
        </div>
    );
}

function Node({ node, handleContextMenu, stateFunctions }) {
    if (!node) {
        return null;
    }
    const children = node.getUrdfObjectChildren();
    const name = node.name;

    // Display the current node's data and render its children
    return (
        <div style={{ marginLeft: "20px" }}>
            {
                <button
                    className="tree-item"
                    onClick={() => {
                        stateFunctions.selectObject(node);
                    }}
                    onContextMenu={(e) => {
                        handleContextMenu(e, node);
                    }}
                >
                    {name}
                </button>
            }
            {children && (
                <>
                    {children.map((child) => (
                        <Node node={child} handleContextMenu={handleContextMenu} stateFunctions={stateFunctions} />
                    ))}
                </>
            )}
        </div>
    );
}
