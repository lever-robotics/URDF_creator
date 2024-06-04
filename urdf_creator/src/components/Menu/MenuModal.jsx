import * as React from 'react';
import { useState } from 'react';
import Button from '@mui/material/Button';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import { styled } from '@mui/material/styles';
import PopupState, { bindTrigger, bindMenu } from 'material-ui-popup-state';
import './MenuModal.css'

export default function MenuModal() {
  const [projectTitle, setProjectTitle] = useState('untitled');

  const StyledMenuItem = styled(props => (
    <MenuItem {...props} disableRipple/>
  ))(({ theme }) => ({
    fontSize: "3em",
    fontWeight: 200,
    fontFamily: "inherit",
    transition: 'font-size 0.4s',
    '&:hover, &.Mui-focusVisible':{
      fontSize: '4em',
    },
  }));

  const StyledMenu = styled((props) => (
    <Menu
      {...props}
    />
  ))(({ theme }) => ({
    '& .MuiPaper-root': {
      background: 'transparent',
      color: 'white',
      boxShadow: 'none',
      height: '100%',
      width: '30%',
    },
    '& .MuiList-root': {
      textOverflow: 'ellipsis',
      height: '90%',
      display: 'flex',
      paddingLeft: '15%',
      justifyContent: 'space-evenly',
      flexDirection: 'column',
    },
    background:'rgba(0, 0, 0, 0.6)',
  }));

  return (
    <PopupState variant="popover" popupId="demo-popup-menu">
      {(popupState) => (
        <React.Fragment>
          <div className='menu'>
            <Button variant="contained" {...bindTrigger(popupState)} class="material-symbols-outlined">
              menu
            </Button>
            <div Id='projectTitle'>{projectTitle}</div>
          </div>
          <StyledMenu {...bindMenu(popupState)}>
            <StyledMenuItem onClick={popupState.close}>Profile</StyledMenuItem>
            <StyledMenuItem onClick={popupState.close}>My account</StyledMenuItem>
            <StyledMenuItem onClick={popupState.close}>Logout</StyledMenuItem>
            <StyledMenuItem onClick={popupState.close}>Another Button</StyledMenuItem>
          </StyledMenu>
        </React.Fragment>
      )}
    </PopupState>
  );
}

           
           
