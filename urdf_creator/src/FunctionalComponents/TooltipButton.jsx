import React, { useRef, useState } from "react";
import Tooltip from "./Tooltip";
import "./TooltipButton.css";

/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function TooltipButton({ className, content = "", anchorPosition = "center", onClick, label, children }) {
    const mouseOver = useRef(false);
    const [showTooltip, setShowTooltip] = useState(false);
    const timeSinceLastMove = useRef(0);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const delay = 300;
    console.log("classname",className);

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
            <button className={className} onMouseMove={handleMouseMove} onMouseLeave={handleMouseLeave} onClick={onClick} Id={label}>
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
