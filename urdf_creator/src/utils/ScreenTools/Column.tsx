import React, { ReactNode } from "react";

type Props = {
    children: ReactNode;
    width: string;
    height: string;
    pointerEvents: "auto" | undefined;
};

const Column: React.FC<Props> = ({ children, width = "", height = "", pointerEvents = "auto" }) => {
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
};

export default Column;
