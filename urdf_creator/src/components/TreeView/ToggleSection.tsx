import { faCaretDown, faCaretRight } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import type React from "react";
import { useState } from "react";
import styles from "./ToggleSection.module.css";

// TODO wrap this in a forward ref
type ToggleSectionProps = {
    children: React.ReactNode;
    renderChildren: () => React.ReactNode;
    renderProperties: () => React.ReactNode;
    isSelected: boolean;
    isHovered: () => boolean;
};
function ToggleSection({
    children,
    renderChildren,
    renderProperties,
    isSelected,
    isHovered,
}: ToggleSectionProps) {
    // const [visible, setVisible] = useState(isSelected());

    // const handleClick = (e: React.MouseEvent) => {
    //     if (visible) {
    //         setVisible(false);
    //     } else {
    //         setVisible(true);
    //     }
    // };

    const toggleSectionStyle = isSelected
        ? {
              borderColor: "#646cff",
          }
        : {};

    const headerSelectedStyle = () => {
        if (isSelected) {
            return {
                borderColor: "#646cff",
                backgroundColor: "#646cff",
            };
        }
        if (isHovered()) {
            return {
                borderColor: "#646cff",
            };
        }
        return {};
    };

    const iconStyle = isSelected
        ? {
              backgroundColor: "#646cff",
          }
        : {};

    return (
        <div className={styles.toggleSection} style={toggleSectionStyle}>
            <div className={styles.header} style={headerSelectedStyle()}>
                {/* <ToggleIcon
                    onClick={handleClick}
                    visible={visible}
                    selectedStyle={iconStyle}
                /> */}
                {children}
            </div>
            {isSelected && renderProperties() && (
                <div className={styles.body}>{renderProperties()}</div>
            )}
            {/* {visible && renderChildren()} */}
            {renderChildren()}
        </div>
    );
}

export default ToggleSection;

type ToggleIconProps = {
    onClick: (e: React.MouseEvent) => void;
    visible: boolean;
    selectedStyle?: React.CSSProperties;
};
function ToggleIcon(props: ToggleIconProps) {
    return (
        <button
            className={styles.toggleIcon}
            onClick={props.onClick}
            style={props.selectedStyle}
            type="button"
        >
            <FontAwesomeIcon
                icon={props.visible ? faCaretDown : faCaretRight}
            />
        </button>
    );
}
