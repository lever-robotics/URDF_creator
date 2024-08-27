import React from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import './MenuModal.css'; // Assuming you have a CSS file for custom styles

const MenuIcon = ({ openMainMenu }) => {
    const handleClick = () => {
        openMainMenu();
    };

    return (
        <button className="menu-icon-button" onClick={handleClick}>
            <FontAwesomeIcon icon={faBars} />
        </button>
    );
};

export default MenuIcon;