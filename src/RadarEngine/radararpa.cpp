#include "radararpa.h"
#include "radarengine.h"
#include <Crypto/crypto.h>

using namespace RadarEngineARND;

RadarArpa::RadarArpa(QObject *parent,RadarEngine *ri) :
    QObject(parent),m_ri(ri)
{
    cur_elapsed_time = Crypto::initProtect();
    if(!Crypto::checkProtect(cur_elapsed_time))
    {
        qDebug()<<"not valid";
        exit(0);
    }

    m_number_of_targets = 0;
    for (int i = 0; i < MAX_NUMBER_OF_TARGETS; i++)
        m_target[i] = 0;
}

void RadarArpa::RefreshArpaTargets()
{
    //    qDebug()<<Q_FUNC_INFO<<"range_meters"<<range_meters;

    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (m_target[i])
        {
            m_target[i]->m_range = range_meters;
            if (m_target[i]->m_status == LOST)
            {
                qDebug()<<Q_FUNC_INFO<<"lost target "<<i;
                // we keep the lost target for later use, destruction and construction is expensive
                ARPATarget* lost = m_target[i];
                int len = sizeof(ARPATarget*);
                // move rest of larget list up to keep them in sequence
                memmove(&m_target[i], &m_target[i] + 1, (m_number_of_targets - i) * len);
                m_number_of_targets--;
                // set the lost target at the last position
                m_target[m_number_of_targets] = lost;
            }
        }
    }

    int target_to_delete = -1;
    // find a target with status FOR_DELETION if it is there
    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (!m_target[i]) continue;
        if (m_target[i]->m_status == FOR_DELETION)
        {
            target_to_delete = i;
        }
    }
    if (target_to_delete != -1)
    {
        // delete the target that is closest to the target with status FOR_DELETION
        Position* deletePosition = &m_target[target_to_delete]->m_position;
        double min_dist = 1000;
        int del_target = -1;
        for (int i = 0; i < m_number_of_targets; i++)
        {
            if (!m_target[i]) continue;
            if (i == target_to_delete || m_target[i]->m_status == LOST) continue;
            double dif_lat = deletePosition->lat - m_target[i]->m_position.lat;
            double dif_lon = (deletePosition->lon - m_target[i]->m_position.lon) * cos(deg2rad(deletePosition->lat));
            double dist2 = dif_lat * dif_lat + dif_lon * dif_lon;
            if (dist2 < min_dist)
            {
                min_dist = dist2;
                del_target = i;
            }
        }
        // del_target is the index of the target closest to target with index target_to_delete
        if (del_target != -1)
        {
            qDebug()<<Q_FUNC_INFO<<"set lost closest";
            m_target[del_target]->SetStatusLost();
        }
        m_target[target_to_delete]->SetStatusLost();
    }
    //    qDebug()<<Q_FUNC_INFO<<"target to delete "<<target_to_delete<<"target number"<<m_number_of_targets;

    // main target refresh loop

    // pass 1 of target refresh
    int dist = arpa_settings.search_radius1;
    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (!m_target[i])
        {
            qDebug()<<Q_FUNC_INFO<<"BR24radar_pi:  error target non existent i="<<i;
            continue;
        }
        m_target[i]->m_pass_nr = PASS1;
        if (m_target[i]->m_pass1_result == NOT_FOUND_IN_PASS1) continue;
        m_target[i]->RefreshTarget(dist);
        /*
        if (m_target[i]->m_pass1_result == NOT_FOUND_IN_PASS1)
        {
        }
        */
    }

    // pass 2 of target refresh
    dist = arpa_settings.search_radius2;
    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (!m_target[i])
        {
            qDebug()<<Q_FUNC_INFO<<" error target non existent i="<<i;
            continue;
        }
        if (m_target[i]->m_pass1_result == UNKNOWN) continue;
        m_target[i]->m_pass_nr = PASS2;
        m_target[i]->RefreshTarget(dist);
    }
    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (!m_target[i])
        {
            qDebug()<<Q_FUNC_INFO<<" error target non existent i="<<i;
            continue;
        }
    }

}
bool RadarArpa::Pix(int ang, int rad)
{
    if (rad <= 1 || rad >= RETURNS_PER_LINE - 1) //  avoid range ring
        return false;

    //    qDebug()<<Q_FUNC_INFO<<ang<<rad;
    return ((m_ri->m_history[MOD_ROTATION2048(ang)].line[rad] & 128) != 0);
}

bool RadarArpa::MultiPix(int ang, int rad)
{
    // checks the blob has a contour of at least length pixels
    // pol must start on the contour of the blob
    // false if not
    // if false clears out pixels of th blob in hist
    //    wxCriticalSectionLocker lock(ArpaTarget::m_ri->m_exclusive);
    int length = arpa_settings.min_contour_length;
    Polar start;
    start.angle = ang;
    start.r = rad;
    if (!Pix(start.angle, start.r))
        return false;

    Polar current = start;  // the 4 possible translations to move from a point on the contour to the next
    Polar max_angle;
    Polar min_angle;
    Polar max_r;
    Polar min_r;
    Polar transl[4];  //   = { 0, 1,   1, 0,   0, -1,   -1, 0 };
    transl[0].angle = 0;
    transl[0].r = 1;
    transl[1].angle = 1;
    transl[1].r = 0;
    transl[2].angle = 0;
    transl[2].r = -1;
    transl[3].angle = -1;
    transl[3].r = 0;
    int count = 0;
    int aa;
    int rr;
    bool succes = false;
    int index = 0;
    max_r = current;
    max_angle = current;
    min_r = current;
    min_angle = current;  // check if p inside blob
    if (start.r >= RETURNS_PER_LINE - 1)
        return false;  //  r too large

    if (start.r < 3)
        return false;  //  r too small

    // first find the orientation of border point p
    for (int i = 0; i < 4; i++)
    {
        index = i;
        aa = current.angle + transl[index].angle;
        rr = current.r + transl[index].r;
        succes = !Pix(aa, rr);
        if (succes) break;
    }
    if (!succes)
    {
        qDebug()<<Q_FUNC_INFO<<" Error starting point not on contour";
        return false;
    }
    index += 1;  // determines starting direction
    if (index > 3) index -= 4;
    while (current.r != start.r || current.angle != start.angle || count == 0)
    {  // try all translations to find the next point  // start with the "left most" translation relative to the
        // previous one
        index += 3;  // we will turn left all the time if possible
        for (int i = 0; i < 4; i++)
        {
            if (index > 3) index -= 4;
            aa = current.angle + transl[index].angle;
            rr = current.r + transl[index].r;
            succes = Pix(aa, rr);
            if (succes)   // next point found
                break;

            index += 1;
        }
        if (!succes)
        {
            qDebug()<<Q_FUNC_INFO<<"RA::CheckContour no next point found count="<<count;
            return false;  // return code 7, no next point found
        }                // next point found
        current.angle = aa;
        current.r = rr;
        if (count >= length)
            return true;

        count++;
        if (current.angle > max_angle.angle)
            max_angle = current;

        if (current.angle < min_angle.angle)
            min_angle = current;

        if (current.r > max_r.r)
            max_r = current;

        if (current.r < min_r.r)
            min_r = current;

    }  // contour length is less than m_min_contour_length
    // before returning false erase this blob so we do not have to check this one again
    if (min_angle.angle < 0)
    {
        min_angle.angle += LINES_PER_ROTATION;
        max_angle.angle += LINES_PER_ROTATION;
    }
    for (int a = min_angle.angle; a <= max_angle.angle; a++)
    {
        for (int r = min_r.r; r <= max_r.r; r++)
            m_ri->m_history[MOD_ROTATION2048(a)].line[r] &= 63;
    }
    return false;
}
void RadarArpa::DeleteAllTargets()
{
    for (int i = 0; i < m_number_of_targets; i++)
    {
        if (!m_target[i]) continue;
        m_target[i]->SetStatusLost();
    }
}
void RadarArpa::AcquireNewMARPATarget(Position p)
{
    //    qDebug()<<Q_FUNC_INFO<<p.lat<<p.lon;

    AcquireOrDeleteMarpaTarget(p, ACQUIRE0);
}
void RadarArpa::AcquireOrDeleteMarpaTarget(Position target_pos, int status)
{
    //    qDebug()<<Q_FUNC_INFO<<target_pos.lat<<target_pos.lon<<m_number_of_targets;
    int i_target;
    if (m_number_of_targets < MAX_NUMBER_OF_TARGETS - 1 ||
            (m_number_of_targets == MAX_NUMBER_OF_TARGETS - 1 && status == FOR_DELETION))
    {
        if (m_target[m_number_of_targets] == 0)
        {
            m_target[m_number_of_targets] = new ARPATarget(this,m_ri);
            //            qDebug()<<Q_FUNC_INFO<<"create new ARPAtarget";
        }
        i_target = m_number_of_targets;
        m_number_of_targets++;
    }
    else
    {
        qDebug()<<Q_FUNC_INFO<<" Error, max targets exceeded ";
        return;
    }

    ARPATarget* target = m_target[i_target];
    target->m_position = target_pos;  // Expected position
    target->m_position.time = QDateTime::currentMSecsSinceEpoch();
    target->m_position.dlat_dt = 0.;
    target->m_position.dlon_dt = 0.;
    target->m_status = status;
    qDebug()<<Q_FUNC_INFO<<"target new status"<<status;
    qDebug()<<Q_FUNC_INFO<<"target lat"<<target->m_position.lat;
    qDebug()<<Q_FUNC_INFO<<"target lon"<<target->m_position.lon;

    target->m_max_angle.angle = 0;
    target->m_min_angle.angle = 0;
    target->m_max_r.r = 0;
    target->m_min_r.r = 0;
    target->m_max_r_future.angle = 0;
    target->m_min_r_future.angle = 0;
    target->m_max_r_future.r = 0;
    target->m_min_r_future.r = 0;
    target->future_first = true;

    if (!target->m_kalman)
        target->m_kalman = new KalmanFilter(this);

    target->m_automatic = false;
    return;

}
int RadarArpa::AcquireNewARPATarget(Polar pol, int status)
{
    // acquires new target from mouse click position
    // no contour taken yet
    // target status status, normally 0, if dummy target to delete a target -2
    // returns in X metric coordinates of click
    // constructs Kalman filter
    Position own_pos;
    Position target_pos;
    own_pos.lat = currentOwnShipLat;
    own_pos.lon =currentOwnShipLon;
    target_pos = Polar2Pos(pol, own_pos, range_meters);
    // make new target or re-use an existing one with status == lost
    qDebug()<<Q_FUNC_INFO<<"range_meters"<<range_meters;
    int i;
    if (m_number_of_targets < MAX_NUMBER_OF_TARGETS - 1 || (m_number_of_targets == MAX_NUMBER_OF_TARGETS - 1 && status == -2))
    {
        if (!m_target[m_number_of_targets])
            m_target[m_number_of_targets] = new ARPATarget(this, m_ri);

        i = m_number_of_targets;
        m_number_of_targets++;
    }
    else
    {
        qDebug()<<Q_FUNC_INFO<<" Error, max targets exceeded ";
        return -1;
    }

    ARPATarget* target = m_target[i];

    target->m_position = target_pos;  // Expected position
    target->m_position.time = QDateTime::currentMSecsSinceEpoch();
    target->m_position.dlat_dt = 0.;
    target->m_position.dlon_dt = 0.;
    target->m_position.sd_speed_kn = 0.;
    target->m_status = status;
    target->m_max_angle.angle = 0;
    target->m_min_angle.angle = 0;
    target->m_max_r.r = 0;
    target->m_min_r.r = 0;
    target->m_max_r_future.angle = 0;
    target->m_min_r_future.angle = 0;
    target->m_max_r_future.r = 0;
    target->m_min_r_future.r = 0;
    target->future_first = true;

    if (!target->m_kalman)
        target->m_kalman = new KalmanFilter();

    target->m_check_for_duplicate = false;
    target->m_automatic = true;
    target->m_target_id = 0;
    target->RefreshTarget(arpa_settings.search_radius1);
    return i;
}

void RadarArpa::DeleteTarget(Position target_pos) { AcquireOrDeleteMarpaTarget(target_pos, FOR_DELETION); }

