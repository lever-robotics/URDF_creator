import {
    faMaximize,
    faRotate,
    faRotateLeft,
    faRotateRight,
    faUpDownLeftRight,
} from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import type React from "react";
import { type MutableRefObject, useEffect, useState } from "react";
import TooltipButton from "../../FunctionalComponents/TooltipButton";
import VisualCollision, {
    Collision,
    Visual,
} from "../../Models/VisualCollision";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import type {
    Selectable,
    TransformControlsMode,
} from "../ThreeDisplay/ThreeScene";

const Toolbar = ({
    threeSceneRef,
    // popUndo,
    // popRedo,
}: {
    threeSceneRef: MutableRefObject<ThreeScene | undefined>;
    // popUndo: () => void;
    // popRedo: () => void;
}) => {
    const threeScene = threeSceneRef.current;
    const [selectedObject, setSelectedObject] = useState<Selectable>();
    const [toolMode, setToolMode] = useState<TransformControlsMode>();

    // useEffect(() => {
    //     setSelectedObject(threeScene.selectedObject);
    //     setToolMode(threeScene.toolMode);
    // }, [threeScene.selectedObject, threeScene.toolMode]);

    useEffect(() => {
        if (!threeScene) return;

        const handleToolModeChange = () => {
            setToolMode(threeScene.toolMode);
        };

        const handleSelectedObjectChange = () => {
            setSelectedObject(threeScene.selectedObject);
        };

        handleToolModeChange();
        handleSelectedObjectChange();

        threeScene.addEventListener("toolMode", handleToolModeChange);
        threeScene.addEventListener(
            "selectedObject",
            handleSelectedObjectChange,
        );

        return () => {
            threeScene.removeEventListener("toolMode", handleToolModeChange);
            threeScene.removeEventListener(
                "selectedObject",
                handleSelectedObjectChange,
            );
        };
    }, [threeScene]);

    const handleClick = (e: React.MouseEvent<Element>) => {
        if (!threeScene) return;
        const mode = e.currentTarget.id;
        threeScene.setToolMode(mode as TransformControlsMode);
    };

    const canScale = selectedObject instanceof VisualCollision;

    // .row-spaced {
    //     display: flex;
    //     gap: 10px;
    // }

    // .row-space-between {
    //     display: flex;
    //     flex-direction: row;
    //     justify-content: space-between;
    // }

    if (!threeSceneRef.current) return null;

    return (
        <div
            style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }}
            className="row-space-between"
        >
            <div className="row-spaced">
                {/* <TooltipButton
                    onClick={popUndo}
                    content={"Undo (ctrl + z)"}
                    anchorPosition={"top-right"}
                    label="undo"
                    active={false}
                >
                    <FontAwesomeIcon icon={faRotateLeft} />
                </TooltipButton>
                <TooltipButton
                    onClick={popRedo}
                    content={"Redo (ctrl + shift + z)"}
                    anchorPosition={"top-right"}
                    label="redo"
                    active={false}
                >
                    <FontAwesomeIcon icon={faRotateRight} />
                </TooltipButton> */}
                <TooltipButton
                    active={toolMode === "translate"}
                    onClick={handleClick}
                    content={"Translate"}
                    anchorPosition={"top-right"}
                    label="translate"
                >
                    <FontAwesomeIcon icon={faUpDownLeftRight} />
                </TooltipButton>
                <TooltipButton
                    active={toolMode === "rotate"}
                    onClick={handleClick}
                    content={"Rotate"}
                    anchorPosition={"top-right"}
                    label="rotate"
                >
                    <FontAwesomeIcon icon={faRotate} />
                </TooltipButton>
                {canScale && (
                    <TooltipButton
                        active={toolMode === "scale"}
                        onClick={handleClick}
                        content={"Scale"}
                        anchorPosition={"top-right"}
                        label="scale"
                    >
                        <FontAwesomeIcon icon={faMaximize} />
                    </TooltipButton>
                )}
            </div>
        </div>
    );
};

export default Toolbar;
