import React, { ReactNode, useRef, useState } from "react";
import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./TreeFrame.module.css";
import ToggleSection from "./ToggleSection";
import TreeProperty from "./TreeProperty";
import { ContextMenu, Property } from "./LinkTree";

type Props = {
    frame: Frame;
    handleContextMenu: (e: React.MouseEvent, treeObject: ContextMenu) => void;
    threeScene: ThreeScene;
    hoveredFrame: Frameish;
    setHoveredFrame: (f: Frameish) => void;
};

export default function TreeFrame(props: Props) {
    // Destructure props
    const { frame, ...restProps } = props;
    const { handleContextMenu, threeScene, hoveredFrame, setHoveredFrame } =
        restProps;

    const onClick = () => {
        threeScene.selectObject(frame);
    }

    const onContextMenu = (e: React.MouseEvent) => {
        handleContextMenu(e, frame);
    }

    const onDragEnter = () => {
        setHoveredFrame(frame);
    };

    // put the button that is dragged as the child of the hovered button
    const onDragEnd = (e: React.MouseEvent) => {
        if (frame.isRootFrame) return; // Do not let the rootFrame be reparented

        if (hoveredFrame) {
            if (!isAncestor(frame, hoveredFrame)) {
                threeScene?.reparentObject(hoveredFrame, frame);
                threeScene?.selectObject(frame);
                setHoveredFrame(null);
            }
        }
    };

    const renderChildren = () => {
        const children = frame.getFrameChildren();
        return children.map((child: Frame) => (
            <TreeFrame key={child.id} frame={child} {...restProps} />
        ));
    };

    const renderProperties = () => {
        // const properties = frame.getProperties();
        // return properties.map((property: any) => (
        //     <TreeProperty key={property.id} property={property} {...restProps}/>
        // ));
        const properties = [{name: "Visual", id: 1}, {name: "Collision", id: 2}, {name: "Inertial", id: 3}];
        return properties.map((property: Property) => (
            <TreeProperty key={property.id} property={property} {...restProps}/>
        ));
    }

    const isSelected = () => {
        const selectedObject = threeScene.selectedObject;
        if (!selectedObject) return false;
        if (selectedObject.name === frame.name) return true;
        return false;
    };

    const isHovered = () => {
        if(hoveredFrame){
            if(hoveredFrame.name === frame.name) return true;
        }
        return false;
    }

    const selectedStyle = isSelected() ? {backgroundColor: "#646cff"}: {};

    // Display the current node's data and render its children
    return (
        <ToggleSection renderChildren={renderChildren} renderProperties={renderProperties} isSelected={isSelected} isHovered={isHovered}>
                <button
                    key={frame.id}
                    className={styles.treeFrame}
                    style={selectedStyle}
                    // className={className}
                    onContextMenu={onContextMenu}
                    draggable={true}
                    onClick={onClick}
                    onDragEnd={onDragEnd}
                    onDragEnter={onDragEnter}>
                    {frame.name}
                </button>
        </ToggleSection>
    );
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
