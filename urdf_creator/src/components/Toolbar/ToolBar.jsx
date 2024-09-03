import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faRotateLeft, faRotateRight, faUpDownLeftRight, faMaximize, faRotate } from "@fortawesome/free-solid-svg-icons";
import TooltipButton from "../../FunctionalComponents/TooltipButton";

const Toolbar = ({ selectedObject, stateFunctions }) => {
    const handleClick = (e) => {
        const mode = e.target.innerText.toLowerCase();
        stateFunctions.setTransformMode(selectedObject, mode);
    };

    const tool = stateFunctions.getToolMode();

    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <TooltipButton onClick={stateFunctions.popUndo} content={"Undo (ctrl + z)"} anchorPosition={"top-right"}>
                    <FontAwesomeIcon icon={faRotateLeft} />
                </TooltipButton>
                <TooltipButton onClick={stateFunctions.popRedo} content={"Redo (ctrl + shift + z)"} anchorPosition={"top-right"}>
                    <FontAwesomeIcon icon={faRotateRight} />
                </TooltipButton>
                <TooltipButton className={tool === "translate" ? "button_selected" : ""} onClick={handleClick} content={"Translate"} anchorPosition={"top-right"}>
                    <FontAwesomeIcon icon={faUpDownLeftRight} />
                </TooltipButton>
                <TooltipButton className={stateFunctions.getToolMode() === "rotate" ? "button_selected" : ""} onClick={handleClick} content={"Rotate"} anchorPosition={"top-right"}>
                    <FontAwesomeIcon icon={faRotate} />
                </TooltipButton>
                <TooltipButton className={stateFunctions.getToolMode() === "scale" ? "button_selected" : ""} onClick={handleClick} content={"Scale"} anchorPosition={"top-right"}>
                    <FontAwesomeIcon icon={faMaximize} />
                </TooltipButton>
            </div>
        </div>
    );
};

export default Toolbar;
