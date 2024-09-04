import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faRotateLeft, faRotateRight, faUpDownLeftRight, faMaximize, faRotate } from "@fortawesome/free-solid-svg-icons";
import TooltipButton from "../../FunctionalComponents/TooltipButton";
import React from "react";

const Toolbar = ({ selectedObject, stateFunctions, toolMode }) => {
    const handleClick = (e) => {
        const mode = e.currentTarget.id;
        stateFunctions.setTransformMode(selectedObject, mode);
    };


    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <TooltipButton onClick={stateFunctions.popUndo} content={"Undo (ctrl + z)"} anchorPosition={"top-right"} label="undo">
                    <FontAwesomeIcon icon={faRotateLeft} />
                </TooltipButton>
                <TooltipButton onClick={stateFunctions.popRedo} content={"Redo (ctrl + shift + z)"} anchorPosition={"top-right"} label="redo">
                    <FontAwesomeIcon icon={faRotateRight} />
                </TooltipButton>
                <TooltipButton active={toolMode === "translate"} onClick={handleClick} content={"Translate"} anchorPosition={"top-right"} label="translate">
                    <FontAwesomeIcon icon={faUpDownLeftRight} />
                </TooltipButton>
                <TooltipButton active={toolMode === "rotate"} onClick={handleClick} content={"Rotate"} anchorPosition={"top-right"} label="rotate">
                    <FontAwesomeIcon icon={faRotate} />
                </TooltipButton>
                <TooltipButton active={toolMode === "scale"} onClick={handleClick} content={"Scale"} anchorPosition={"top-right"} label="scale">
                    <FontAwesomeIcon icon={faMaximize} />
                </TooltipButton>
            </div>
        </div>
    );
};

export default Toolbar;
