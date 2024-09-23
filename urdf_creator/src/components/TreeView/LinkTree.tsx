import AllClickButton from "../../FunctionalComponents/AllClickButton";
import Frame from "../../Models/Frame";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import { StateFunctionsType } from "../SceneState";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useState } from "react";

// RecursiveTreeView Component
export function LinkTree({ selectedObject, stateFunctions }: ParameterProps) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [contextMenuVisible, setContextMenuVisible] = useState(false);
    const [draggedButton, setDraggedButton] = useState(null);
    const [hoveredButton, setHoveredButton] = useState(null);

    const handleContextMenu = (e: React.MouseEvent, node: Frame) => {
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

    const rootFrame = stateFunctions.getRootFrame();

    // put the button that is dragged as the child of the hovered button
    const dropButton = (e: React.MouseEvent) => {
        if (hoveredButton && draggedButton) {
            if (draggedButton !== hoveredButton && !isAncestor(draggedButton, hoveredButton)) {
                stateFunctions.reparentObject(hoveredButton, draggedButton);
                stateFunctions.selectObject(draggedButton);
                setHoveredButton(null);
            }
        }
        setDraggedButton(null);
    };

    const isAncestor = (ancestor: Frame, descendant: Frame) => {
        if (descendant === ancestor) return true;
        if (ancestor.isRootFrame || descendant.isRootFrame) return false;
        return isAncestor(ancestor, descendant.parentFrame!);
    };

    return (
        <div className="object-tree" onClick={onClick} onMouseLeave={onMouseLeave}>
            Link Tree
            <div className="scroll-box">
                {rootFrame && (
                    <Node
                        node={rootFrame}
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

type Props = { node: Frame | null | undefined, selectedObject: Frame | null | undefined, handleContextMenu: any, stateFunctions: StateFunctionsType, setDraggedButton: any, hoveredButton: any, setHoveredButton: any, dropButton: any }

function Node({ node, selectedObject, handleContextMenu, stateFunctions, setDraggedButton, hoveredButton, setHoveredButton, dropButton }: Props) {
    if (!node) {
        return null;
    }
    const children = node.getFrameChildren();
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
                    onDragLeave={(e: any) => {
                        if (hoveredButton.current === node && e.relatedTarget) {
                            setHoveredButton(null);
                        }
                    }}
                    onDragStart={() => {
                        if (!node.isRootFrame) setDraggedButton(node);
                    }}
                    onDragEnd={dropButton}
                    onContextMenu={(e: any) => {
                        handleContextMenu(e, node);
                    }}
                    draggable={true}
                >
                    {name}
                </AllClickButton>
            }
            {children && (
                <>
                    {children.map((child: Frame) => (
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
