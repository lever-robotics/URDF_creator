import React from "react";

/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function Tooltip({ mousePosition, anchorPosition = "center", children }) {
    console.log(anchorPosition);
    return (
        <div
            className="tooltip"
            style={{
                left: mousePosition.x,
                top: mousePosition.y,
                transform: anchorPosition === "center" ? "translate(-50%, -50%)" : anchorPosition === "top-right" ? "translate(15px, 15px)" : "translate(-50%, -50%)",
            }}
        >
            {children}
        </div>
    );
}
