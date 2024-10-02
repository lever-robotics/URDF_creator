import React, { ReactNode, useRef } from "react";
import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./TreeView.module.css";

type Props = {
    frame: Frameish;
    handleContextMenu: (e: React.MouseEvent, f: Frame) => void;
    threeScene: ThreeScene;
    hoveredButton: Frameish;
    setHoveredButton: (f: Frameish) => void;
};

export default function TreeFrame(props: Props) {
    // Destructure props
    const { frame, ...restProps } = props;
    const { handleContextMenu, threeScene, hoveredButton, setHoveredButton } =
        restProps;

    if (!frame) return null;

    const onClick = () => {
        threeScene.selectObject(frame);
    }

    const onDragEnter = () => {
        setHoveredButton(frame);
    };

    // put the button that is dragged as the child of the hovered button
    const onDragEnd = (e: React.MouseEvent) => {
        if (frame.isRootFrame) return; // Do not let the rootFrame be reparented

        if (hoveredButton) {
            if (!isAncestor(frame, hoveredButton)) {
                threeScene?.reparentObject(hoveredButton, frame);
                threeScene?.selectObject(frame);
                setHoveredButton(null);
            }
        }
    };

    const onContextMenu = (e: React.MouseEvent) => {
        handleContextMenu(e, frame);
    };

    const className = determineClassName(
        threeScene.selectedObject,
        frame,
        hoveredButton
    );

    /**
     * Return TreeFrame components for each child of a frame objcet
     * @returns TreeFrame[]
     */
    const renderChildren = () => {
        const children = frame.getFrameChildren();
        return children.map((child: Frame) => (
            <TreeFrame key={child.id} frame={child} {...restProps} />
        ));
    };

    // Display the current node's data and render its children
    return (
        <div style={{ marginLeft: "20px" }}>
            <button
                key={frame.id}
                className={className}
                onContextMenu={onContextMenu}
                draggable={true}
                onClick={onClick}
                onDragEnd={onDragEnd}
                onDragEnter={onDragEnter}>
                {frame.name}
            </button>
            {renderChildren()}
        </div>
    );
}

/**
 * The treeFrame can have multiple class names for conditional styling. This determines all the classNames
 * @returns string The class names seperated by spaces
 */
function determineClassName(
    selectedObject: Frameish,
    frame: Frame,
    hoveredButton: Frameish
) {
    const isSelected = () => {
        if (!selectedObject) return;
        if (selectedObject.name === frame.name) return styles.selected;
        return;
    };

    console.log("hovered");
    const isHover = () => {
        console.log(hoveredButton === frame);
        if (hoveredButton === frame) return styles.hover;
        return;
    };

    return `${styles.treeFrame} ${isSelected()} ${isHover()}`;
}

/**
 * Determines if a frame is its own ancestor
 * @param ancestor frame
 * @param descendant frame
 * @returns
 */
function isAncestor(ancestor: Frame, descendant: Frame) {
    if (descendant === ancestor) return true;
    if (ancestor.isRootFrame || descendant.isRootFrame) return false;
    return isAncestor(ancestor, descendant.parentFrame!);
}
