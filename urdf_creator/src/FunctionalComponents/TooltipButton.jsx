import React, { useRef, useState } from "react";
import Tooltip from "./Tooltip";

/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function TooltipButton({ content = "", anchorPosition = "center", children }) {
    const mouseOver = useRef(false);
    const [showTooltip, setShowTooltip] = useState(false);
    const timeSinceLastMove = useRef(0);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const delay = 1200;

    function handleMouseMove(e) {
        mouseOver.current = true;
        timeSinceLastMove.current = Date.now();
        setShowTooltip(false);
        setTimeout(() => {
            if (Date.now() - timeSinceLastMove.current >= delay && mouseOver.current) {
                console.log("showing tooltip");
                setMousePos({ x: e.clientX, y: e.clientY });
                setShowTooltip(true);
            }
        }, delay);
    }

    function handleMouseLeave(e) {
        console.log("left");
        mouseOver.current = false;
        setShowTooltip(false);
    }

    return (
        <>
            <button onMouseMove={handleMouseMove} onMouseLeave={handleMouseLeave}>
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
