import AllClickButton from "../../FunctionalComponents/AllClickButton";
import Tooltip from "../../FunctionalComponents/Tooltip";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useRef, useState } from "react";

// RecursiveTreeView Component
export function LinkTree({ selectedObject, stateFunctions }) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [contextMenuVisible, setContextMenuVisible] = useState(false);
    const [draggedButton, setDraggedButton] = useState(null);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const hoveredButton = useRef(null);

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
        setDraggedButton(null);
    };

    const onClick = () => {
        setContextMenuVisible(false);
    };

    const baseLink = stateFunctions.getBaseLink();

    const handleMouseMove = (e) => {
        setMousePos({ x: e.clientX, y: e.clientY });
    };

    // put the button that is dragged as the child of the hovered button
    const handleMouseUp = (e) => {
        if (hoveredButton.current && draggedButton) {
            if (draggedButton !== hoveredButton.current && !isAncestor(draggedButton, hoveredButton.current)) {
                stateFunctions.reparentObject(hoveredButton.current, draggedButton);
                stateFunctions.selectObject(draggedButton);
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
        <div className="object-tree" onClick={onClick} onMouseLeave={onMouseLeave} onMouseMove={handleMouseMove} onMouseUp={handleMouseUp}>
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
            {draggedButton && <Tooltip mousePosition={mousePos}>{draggedButton.name}</Tooltip>}
        </div>
    );
}

function Node({ node, selectedObject, handleContextMenu, stateFunctions, setDraggedButton, hoveredButton }) {
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
                    className={`tree-item ${isSelected ? "button_selected" : "button_unselected"}`}
                    onClick={() => {
                        stateFunctions.selectObject(node);
                    }}
                    onDoubleClick={() => {
                        //put renaming functionality here
                        stateFunctions.selectObject(node);
                    }}
                    onMouseEnter={() => {
                        hoveredButton.current = node;
                    }}
                    onMouseLeave={() => {
                        if (hoveredButton.current === node) {
                            hoveredButton.current = null;
                        }
                    }}
                    onLongClick={() => {
                        if (!node.isBaseLink) setDraggedButton(node);
                    }}
                    onContextMenu={(e) => {
                        handleContextMenu(e, node);
                    }}
                >
                    {name}
                </AllClickButton>
            }
            {children && (
                <>
                    {children.map((child) => (
                        <Node
                            node={child}
                            handleContextMenu={handleContextMenu}
                            selectedObject={selectedObject}
                            stateFunctions={stateFunctions}
                            setDraggedButton={setDraggedButton}
                            hoveredButton={hoveredButton}
                        />
                    ))}
                </>
            )}
        </div>
    );
}
