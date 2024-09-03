import React, { useRef } from "react";

export default function AllClickButton({
    className = {},
    onClick = () => {},
    onDoubleClick = () => {},
    onLongClick = () => {},
    onLongClickUp = () => {},
    onContextMenu = () => {},
    onMouseEnter = () => {},
    onMouseLeave = () => {},
    onMouseUp = () => {},
    children,
}) {
    let clicks = useRef(0);
    let timeAtClick = useRef(0);
    let longClicking = useRef(false);

    const clickTime = 250;

    function handleDown(e) {
        timeAtClick.current = Date.now();
        setTimeout(() => {
            // break out if there has been a click in the last second
            if (Date.now() - timeAtClick.current < clickTime) {
                return;
            }

            switch (clicks.current) {
                case 0:
                    onLongClick(e);
                    longClicking.current = true;
                    break;
                case 1:
                    onClick(e);
                    clicks.current = 0;
                    break;
                // for all clicks 2 or more
                default:
                    onDoubleClick(e);
                    clicks.current = 0;
                    break;
            }
        }, clickTime);
    }

    function handleUp(e) {
        clicks.current += 1;
        onMouseUp(e);
        if (longClicking.current) {
            longClicking.current = false;
            onLongClickUp(e);
            clicks.current = 0;
        }
    }

    return (
        <button onMouseDown={handleDown} onMouseUp={handleUp} onContextMenu={onContextMenu} onMouseEnter={onMouseEnter} onMouseLeave={onMouseLeave} className={className}>
            {children}
        </button>
    );
}
