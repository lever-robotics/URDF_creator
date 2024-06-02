import React from "react";

export default function AbsolutePosition({ children }) {
    return (
        <div
            style={{
                position: "absolute",
                pointerEvents: "none",
                display: "flex",
                width: "100%",
                height: "100%",
                overflow: "hidden",
            }}
        >
            {children}
        </div>
    );
}
