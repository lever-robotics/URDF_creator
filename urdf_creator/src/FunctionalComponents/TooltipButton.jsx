import React, { useRef, useState } from "react";
import Tooltip from "./Tooltip";
import "./TooltipButton.css";

/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function TooltipButton({ active = false, content = "", anchorPosition = "center", onClick, label, children }) {
    const mouseOver = useRef(false);
    const [showTooltip, setShowTooltip] = useState(false);
    const timeSinceLastMove = useRef(0);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const delay = 300;

    function handleMouseMove(e) {
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

    function handleMouseLeave(e) {
        mouseOver.current = false;
        setShowTooltip(false);
    }

    return (
        <>
            <button className={active ? "active" : ""} onMouseMove={handleMouseMove} onMouseLeave={handleMouseLeave} onClick={onClick} Id={label}>
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
