import AllClickButton from "../../FunctionalComponents/AllClickButton";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useState } from "react";

// RecursiveTreeView Component
export function LinkTree({ selectedObject, stateFunctions }) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [contextMenuVisible, setContextMenuVisible] = useState(false);
    const [draggedButton, setDraggedButton] = useState(null);
    const [hoveredButton, setHoveredButton] = useState(null);

    const handleContextMenu = (e, node) => {
        e.preventDefault();
        stateFunctions.selectObject(node);
        setContextMenuVisible(true);
        setContextMenuPosition({
            left: e.clientX,
            top: e.clientY,
        });
    };

    const onMouseLeave = () => {
        setContextMenuVisible(false);
    };

    const onClick = () => {
        setContextMenuVisible(false);
    };

    const baseLink = stateFunctions.getBaseLink();

    // put the button that is dragged as the child of the hovered button
    const dropButton = (e) => {
        if (hoveredButton && draggedButton) {
            if (draggedButton !== hoveredButton && !isAncestor(draggedButton, hoveredButton)) {
                stateFunctions.reparentObject(hoveredButton, draggedButton);
                stateFunctions.selectObject(draggedButton);
                setHoveredButton(null);
            }
        }
        setDraggedButton(null);
    };

    const isAncestor = (ancestor, descendant) => {
        if (descendant === ancestor) return true;
        if (ancestor.isBaseLink || descendant.isBaseLink) return false;
        return isAncestor(ancestor, descendant.parentURDF);
    };

    return (
        <div className="object-tree" onClick={onClick} onMouseLeave={onMouseLeave}>
            Link Tree
            <div className="scroll-box">
                {baseLink && (
                    <Node
                        node={baseLink}
                        selectedObject={selectedObject}
                        handleContextMenu={handleContextMenu}
                        stateFunctions={stateFunctions}
                        setDraggedButton={setDraggedButton}
                        hoveredButton={hoveredButton}
                        setHoveredButton={setHoveredButton}
                        dropButton={dropButton}
                    />
                )}
            </div>
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

function Node({ node, selectedObject, handleContextMenu, stateFunctions, setDraggedButton, hoveredButton, setHoveredButton, dropButton }) {
    if (!node) {
        return null;
    }
    const children = node.getUrdfObjectChildren();
    const name = node.name;

    //check if the node is the selected object
    const isSelected = selectedObject && selectedObject.name === name ? true : false;

    // Display the current node's data and render its children
    return (
        <div style={{ marginLeft: "20px" }}>
            {
                <AllClickButton
                    key={node.id}
                    className={`tree-item ${isSelected ? "button_selected" : "button_unselected"} ${hoveredButton === node ? "hover" : ""}`}
                    onClick={() => {
                        stateFunctions.selectObject(node);
                    }}
                    onDoubleClick={() => {
                        //put renaming functionality here
                        stateFunctions.selectObject(node);
                    }}
                    onDragEnter={() => {
                        setHoveredButton(node);
                    }}
                    onDragLeave={(e) => {
                        if (hoveredButton.current === node && e.relatedTarget) {
                            setHoveredButton(null);
                        }
                    }}
                    onDragStart={() => {
                        if (!node.isBaseLink) setDraggedButton(node);
                    }}
                    onDragEnd={dropButton}
                    onContextMenu={(e) => {
                        handleContextMenu(e, node);
                    }}
                    draggable={true}
                >
                    {name}
                </AllClickButton>
            }
            {children && (
                <>
                    {children.map((child) => (
                        <Node
                            key={child.id} 
                            node={child}
                            handleContextMenu={handleContextMenu}
                            selectedObject={selectedObject}
                            stateFunctions={stateFunctions}
                            setDraggedButton={setDraggedButton}
                            hoveredButton={hoveredButton}
                            setHoveredButton={setHoveredButton}
                            dropButton={dropButton}
                        />
                    ))}
                </>
            )}
        </div>
    );
}
