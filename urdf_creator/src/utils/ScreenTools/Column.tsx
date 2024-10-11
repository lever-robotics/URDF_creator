import type React from "react";
import type { ReactNode } from "react";
import styles from "./ScreenTools.module.css";

type Props = {
    children: ReactNode;
    width: string;
    height: string;
    pointerEvents: "auto" | undefined;
};

const Column: React.FC<Props> = ({
    children,
    width = "",
    height = "",
    pointerEvents = "auto",
}) => {
    return (
        <div
            className={styles.column}
            style={{
                maxWidth: width === "" ? "auto" : width,
                height: height === "" ? "auto" : height,
            }}
        >
            {children}
        </div>
    );
};

export default Column;
