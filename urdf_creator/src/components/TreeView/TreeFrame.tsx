import type React from "react";
import { ReactNode, useEffect, useRef, useState } from "react";
import Frame, { type Frameish } from "../../Models/Frame";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import type { ContextMenu } from "./LinkTree";
import ToggleSection from "./ToggleSection";
import styles from "./TreeFrame.module.css";
import TreeProperty from "./TreeProperty";

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

    const [selected, setSelected] = useState(false);
    const [name, setName] = useState(frame.name);

    useEffect(() => {
        const updateSelected = () => {
            if (isSelected(threeScene, frame)) {
                setSelected(true);
            } else {
                setSelected(false);
            }
        };

        const updateName = () => {
            if (selected) {
                setName(frame.name);
            }
        };

        updateSelected();
        threeScene.addEventListener("selectedObject", updateSelected);
        threeScene.addEventListener("name", updateName);

        return () => {
            threeScene.removeEventListener("selectedObject", updateSelected);
            threeScene.removeEventListener("name", updateName);
        };
    }, [threeScene, frame, selected]);

    const onClick = () => {
        threeScene.selectObject(frame);
    };

    const onContextMenu = (e: React.MouseEvent) => {
        handleContextMenu(e, frame);
    };

    const onDragEnter = () => {
        setHoveredFrame(frame);
    };

    // put the button that is dragged as the child of the hovered button
    const onDragEnd = (e: React.MouseEvent) => {
        if (frame.isWorldFrame) return; // Do not let the world frame to be reparented

        if (hoveredFrame) {
            if (!isAncestor(frame, hoveredFrame)) {
                threeScene?.reparentObject(hoveredFrame, frame);
                threeScene?.selectObject(frame);
                setHoveredFrame(null);
            }
        }
    };

    const renderChildren = () => {
        if (frame.isWorldFrame) return; // Do not render children of the world frame
        const children = frame.getFrameChildren();
        return children.map((child: Frame) => (
            <TreeFrame key={child.id} frame={child} {...restProps} />
        ));
    };

    const renderProperties = () => {
        if (frame.isWorldFrame) return null; // Do not render properties of the world frame
        const visuals = frame.visuals.map((property) => (
            <TreeProperty
                key={property.id}
                property={property}
                {...restProps}
            />
        ));
        const collisions = frame.collisions.map((property) => (
            <TreeProperty
                key={property.id}
                property={property}
                {...restProps}
            />
        ));

        return visuals.concat(collisions);
        // const properties = [{name: "Visual", id: 1}, {name: "Collision", id: 2}, {name: "Inertial", id: 3}];
        // return properties.map((property: Property) => (
        //     <TreeProperty key={property.id} property={property} {...restProps}/>
        // ));
    };

    const isHovered = () => {
        if (hoveredFrame) {
            if (hoveredFrame.name === frame.name) return true;
        }
        return false;
    };

    const selectedStyle = selected ? { backgroundColor: "#646cff" } : {};

    const draggable = !frame.isWorldFrame;

    // Display the current node's data and render its children
    return (
        <ToggleSection
            renderChildren={renderChildren}
            renderProperties={renderProperties}
            isSelected={selected}
            isHovered={isHovered}
        >
            <button
                key={frame.id}
                className={styles.treeFrame}
                style={selectedStyle}
                // className={className}
                onContextMenu={onContextMenu}
                draggable={draggable}
                onClick={onClick}
                onDragEnd={onDragEnd}
                onDragEnter={onDragEnter}
                type="button"
            >
                {name}
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
    return isAncestor(ancestor, descendant.parentFrame);
}

const isSelected = (threeScene: ThreeScene, frame: Frame) => {
    const selectedObject = threeScene.selectedObject;
    if (!selectedObject) return false;
    if (selectedObject instanceof Frame) {
        if (selectedObject.name === frame.name) return true;
    } else {
        if (selectedObject.frame.name === frame.name) return true;
    }
    return false;
};
