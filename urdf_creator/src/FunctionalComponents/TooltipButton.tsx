import React, { ReactNode, useRef, useState } from "react";
import Tooltip from "./Tooltip";
import styles from "./TooltipButton.module.css";

type Props = {
    active: boolean,
    content: string,
    anchorPosition: string,
    onClick: (e: React.MouseEvent) => void,
    label: string,
    children: ReactNode
}



/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function TooltipButton({ active = false, content = "", anchorPosition = "center", onClick, label, children }: Props) {
    const mouseOver = useRef(false);
    const [showTooltip, setShowTooltip] = useState(false);
    const timeSinceLastMove = useRef(0);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const delay = 300;

    function handleMouseMove(e: React.MouseEvent) {
        mouseOver.current = true;
        timeSinceLastMove.current = Date.now();
        setShowTooltip(false);
        setTimeout(() => {
            if (Date.now() - timeSinceLastMove.current >= delay && mouseOver.current) {
                setMousePos({ x: e.clientX, y: e.clientY });
                setShowTooltip(true);
            }
        }, delay);
    }

    function handleMouseLeave() {
        mouseOver.current = false;
        setShowTooltip(false);
    }

    const isActive = active ? styles.active : "";

    return (
        <>
            <button className={`${styles.button} ${isActive}`} onMouseMove={handleMouseMove} onMouseLeave={handleMouseLeave} onClick={onClick} id={label}>
                {children}
            </button>
            {showTooltip && content && (
                <Tooltip mousePosition={mousePos} anchorPosition={anchorPosition}>
                    {content}
                </Tooltip>
            )}
        </>
    );
}
