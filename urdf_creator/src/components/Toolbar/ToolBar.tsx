import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faRotateLeft, faRotateRight, faUpDownLeftRight, faMaximize, faRotate } from "@fortawesome/free-solid-svg-icons";
import TooltipButton from "../../FunctionalComponents/TooltipButton";
import React from "react";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import { useState, useEffect } from "react";
import VisualCollision, { Collision, Visual } from "../../Models/VisualCollision";

const Toolbar = ({ threeScene, popUndo, popRedo }: { threeScene: ThreeScene, popUndo: () => void, popRedo: () => void }) => {
    const [selectedObject, setSelectedObject] = useState(threeScene?.selectedObject);
    const [toolMode, setToolMode] = useState(threeScene?.toolMode);

    useEffect(() => {
        setSelectedObject(threeScene?.selectedObject);
        setToolMode(threeScene?.toolMode);

    }, [JSON.stringify(threeScene?.toolMode), JSON.stringify(threeScene?.selectedObject)]);

    const handleClick = (e: React.MouseEvent<Element>) => {
        const mode = e.currentTarget.id;
        threeScene.setToolMode(mode);
    };

    const canScale = selectedObject instanceof VisualCollision ? true : false;

    // .row-spaced {
    //     display: flex;
    //     gap: 10px;
    // }

    // .row-space-between {
    //     display: flex;
    //     flex-direction: row;
    //     justify-content: space-between;
    // }

    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <TooltipButton onClick={popUndo} content={"Undo (ctrl + z)"} anchorPosition={"top-right"} label="undo" active={false}>
                    <FontAwesomeIcon icon={faRotateLeft} />
                </TooltipButton>
                <TooltipButton onClick={popRedo} content={"Redo (ctrl + shift + z)"} anchorPosition={"top-right"} label="redo" active={false}>
                    <FontAwesomeIcon icon={faRotateRight} />
                </TooltipButton>
                <TooltipButton active={toolMode === "translate"} onClick={handleClick} content={"Translate"} anchorPosition={"top-right"} label="translate">
                    <FontAwesomeIcon icon={faUpDownLeftRight} />
                </TooltipButton>
                <TooltipButton active={toolMode === "rotate"} onClick={handleClick} content={"Rotate"} anchorPosition={"top-right"} label="rotate">
                    <FontAwesomeIcon icon={faRotate} />
                </TooltipButton>
                {canScale && (
                    <TooltipButton active={toolMode === "scale"} onClick={handleClick} content={"Scale"} anchorPosition={"top-right"} label="scale">
                        <FontAwesomeIcon icon={faMaximize} />
                    </TooltipButton>
                )}
            </div>
        </div>
    );
};

export default Toolbar;
