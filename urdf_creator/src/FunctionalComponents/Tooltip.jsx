import React from "react";

/**
 * @param {*} mousePosition
 * @returns JSX component that displays text that can follow the mouse
 */
export default function Tooltip({ mousePosition, children }) {
    return (
        <div className="tooltip" style={{ left: mousePosition.x, top: mousePosition.y }}>
            {children}
        </div>
    );
}
