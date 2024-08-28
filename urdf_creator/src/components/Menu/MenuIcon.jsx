import { useState, useRef } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import MenuModal from '../../FunctionalComponents/MenuModal';

const MenuIcon = () => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const buttonRef = useRef(null);

    const menuItems = [
        { label: "Profile", action: () => alert("Profile clicked!") },
        { label: "Settings", action: () => alert("Settings clicked!") },
        { label: "Logout", action: () => alert("Logout clicked!") },
    ];

    const handleClick = () => {
        if(isModalOpen){
            setIsModalOpen(false);
        }else{
            setIsModalOpen(true);
        }
    };

    return (
        <>
            <button className="menu-icon-button" onClick={handleClick} onBlur={handleClick} ref={buttonRef}>
                <FontAwesomeIcon icon={faBars} />
            </button>
            <MenuModal
            isOpen={isModalOpen}
            onClose={handleClick}
            menuItems={menuItems}
            buttonRef={buttonRef}
            />
        </>
    );
};

export default MenuIcon;