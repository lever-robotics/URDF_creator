import { useState, useRef } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import MenuModal from '../../FunctionalComponents/MenuModal';

const MenuIcon = ({ openProjectManager }) => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const buttonRef = useRef(null);

    const menuItems = [
        { label: "Project Manager", action: () => {
            handleClick()
            openProjectManager()
        } },
        { label: "Import", action: () => alert("Settings clicked!") },
        { label: "Export", action: () => alert("Logout clicked!") },
        // { label: "Settings", action: () => alert("Setting") }
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
            <button className="menu-icon-button" onClick={handleClick} ref={buttonRef}>
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