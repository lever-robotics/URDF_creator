import { faQuestionCircle } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import React from "react";
import "./onboardstyle.css"; // Assuming you have a CSS file for custom styles

const HelpIcon = ({ openOnboarding }: { openOnboarding: () => void }) => {
    const handleClick = () => {
        openOnboarding();
    };

    return (
        <button
            className="help-icon-button"
            onClick={handleClick}
            type="button"
        >
            <FontAwesomeIcon icon={faQuestionCircle} />
        </button>
    );
};

export default HelpIcon;
