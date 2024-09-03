import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faRotateLeft, faRotateRight, faUpDownLeftRight, faMaximize, faRotate } from "@fortawesome/free-solid-svg-icons";
import TooltipButton from "../../FunctionalComponents/TooltipButton";

const Toolbar = ({ selectedObject, stateFunctions }) => {
    const handleClick = (e) => {
        const mode = e.currentTarget.id;
        stateFunctions.setTransformMode(selectedObject, mode);
    };

    const tool = stateFunctions.getToolMode();
    console.log(tool);

    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <TooltipButton onClick={stateFunctions.popUndo} content={"Undo (ctrl + z)"} anchorPosition={"top-right"} label="undo">
                    <FontAwesomeIcon icon={faRotateLeft}/>
                </TooltipButton>
                <TooltipButton onClick={stateFunctions.popRedo} content={"Redo (ctrl + shift + z)"} anchorPosition={"top-right"} label="redo">
                    <FontAwesomeIcon icon={faRotateRight} />
                </TooltipButton>
                <TooltipButton className={tool === "translate" ? "button_selected" : ""} onClick={handleClick} content={"Translate"} anchorPosition={"top-right"} label="translate">
                    <FontAwesomeIcon icon={faUpDownLeftRight} />
                </TooltipButton>
                <TooltipButton className={tool === "rotate" ? "button_selected" : ""} onClick={handleClick} content={"Rotate"} anchorPosition={"top-right"} label="rotate">
                    <FontAwesomeIcon icon={faRotate} />
                </TooltipButton>
                <TooltipButton className={tool === "scale" ? "button_selected" : ""} onClick={handleClick} content={"Scale"} anchorPosition={"top-right"} label="scale">
                    <FontAwesomeIcon icon={faMaximize} />
                </TooltipButton>
            </div>
        </div>
    );
};

export default Toolbar;
