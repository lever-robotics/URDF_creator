import React from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faQuestionCircle } from '@fortawesome/free-solid-svg-icons';
import './onboardstyle.css'; // Assuming you have a CSS file for custom styles

const HelpIcon= ({ openOnboarding }: {openOnboarding: () => void}) => {
    const handleClick = () => {
        openOnboarding();
    };

    return (
        <button className="help-icon-button" onClick={handleClick}>
            <FontAwesomeIcon icon={faQuestionCircle} />
        </button>
    );
};

export default HelpIcon;