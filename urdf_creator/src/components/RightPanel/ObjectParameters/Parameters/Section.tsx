import { ReactNode } from "react";
import styles from "./Parameter.module.css";

const Section = ({ title, children }: {title?: string, children: ReactNode}) => {

    return (
        <div className={styles.section}>
            <div className={styles.title}>{title}</div>
            <div className={styles.sectionChildren}>
                {children}
            </div>
            {/* <hr className="line"></hr> */}
        </div>
    );
};

export default Section;