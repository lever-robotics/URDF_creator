const Toolbar = ({ selectedObject, stateFunctions }) => {

    const handleClick = (e) => {
        const mode = e.target.innerText.toLowerCase();
        stateFunctions.setTransformMode(selectedObject, mode);
    }

    const tool = stateFunctions.getToolMode();

    return (
        <div
            style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }}
            className="row-space-between">
            <div className="row-spaced">
                <button
                    className={tool === "translate" ? "button_selected" : ""}
                    onClick={handleClick}>
                    Translate
                </button>
                <button
                    className={stateFunctions.getToolMode() === "rotate" ? "button_selected" : ""}
                    onClick={handleClick}>
                    Rotate
                </button>
                <button
                    className={stateFunctions.getToolMode() === "scale" ? "button_selected" : ""}
                    onClick={handleClick}>
                    Scale
                </button>
            </div>
        </div>
    );
};

export default Toolbar;
