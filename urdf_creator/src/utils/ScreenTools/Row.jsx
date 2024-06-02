import React from "react";

export default function Row({ children, width = "", height = "" }) {
    return (
        <div
            style={{
                display: "flex",
                flexDirection: "row",
                flexGrow: "1 1 auto",
                width: width === "" ? "auto" : width,
                height: height === "" ? "auto" : height,
                justifyContent: "space-between",
            }}
        >
            {children}
        </div>
    );
}
