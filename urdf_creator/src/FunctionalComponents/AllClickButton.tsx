import React, { ReactNode, useRef } from "react";

type Props = {
    className : string,
    onClick : (e: any) => void,
    onDoubleClick :(e: any) => void,
    onLongClick :(e: any) => void,
    onLongClickUp :(e: any) => void,
    onContextMenu :(e: any) => void,
    onMouseEnter :(e: any) => void,
    onMouseLeave :(e: any) => void,
    onMouseUp :(e: any) => void,
    onDrag :(e: any) => void,
    onDragStart :(e: any) => void,
    onDragEnd :(e: any) => void,
    onDragEnter :(e: any) => void,
    onDragLeave :(e: any) => void,
    draggable :boolean,
    children: ReactNode,
}

export default function AllClickButton({
    className = "",
    onClick = () => {},
    onDoubleClick = () => {},
    onLongClick = () => {},
    onLongClickUp = () => {},
    onContextMenu = () => {},
    onMouseEnter = () => {},
    onMouseLeave = () => {},
    onMouseUp = () => {},
    onDrag = () => {},
    onDragStart = () => {},
    onDragEnd = () => {},
    onDragEnter = () => {},
    onDragLeave = () => {},
    draggable = false,
    children,
}: Props) {
    let clicks = useRef(0);
    let timeAtClick = useRef(0);
    let longClicking = useRef(false);

    const clickTime = 250;

    function handleDown(e: React.MouseEvent<HTMLButtonElement>) {
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

    function handleUp(e: React.MouseEvent<HTMLButtonElement>) {
        clicks.current += 1;
        onMouseUp(e);
        if (longClicking.current) {
            longClicking.current = false;
            onLongClickUp(e);
            clicks.current = 0;
        }
    }

    return (
        <button
            className={className}
            onMouseDown={handleDown}
            onMouseUp={handleUp}
            onContextMenu={onContextMenu}
            onMouseEnter={onMouseEnter}
            onMouseLeave={onMouseLeave}
            draggable={draggable}
            onDrag={onDrag}
            onDragStart={onDragStart}
            onDragEnd={onDragEnd}
            onDragEnter={onDragEnter}
            onDragLeave={onDragLeave}
        >
            {children}
        </button>
    );
}
