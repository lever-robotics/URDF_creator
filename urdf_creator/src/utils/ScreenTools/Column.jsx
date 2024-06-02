import React from "react";

export default function Column({ children, width = "", height = "", pointerEvents = null }) {
    return (
        <div
            style={{
                display: "flex",
                flexDirection: "column",
                flexGrow: 1,
                maxWidth: width === "" ? "auto" : width,
                height: height === "" ? "auto" : height,
                justifyContent: "space-around",
                pointerEvents,
            }}
        >
            {children}
        </div>
    );
}
